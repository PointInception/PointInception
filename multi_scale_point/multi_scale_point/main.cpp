#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl\common\common.h>
#include <pcl/io/pcd_io.h>
#include <gflags\gflags.h>
#include <liblas\liblas.hpp>
#include <liblas\point.hpp>
#include "voxel_center.h"
#include <windows.h>

#include <iostream>
#include <vector>
#include <ctime>

DEFINE_string(las_path, "", "point cloud file (.las) path");
DEFINE_string(output_dir, "", "output dir");
DEFINE_double(output_res, 0.5, "depth 1 output resolution");
DEFINE_int32(output_dep, 4, "number of depth");

double goffset_x = 0.0;
double goffset_y = 0.0;
double goffset_z = 0.0;
int ReadLas(const std::string &file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
	std::ifstream ifs;
	ifs.open(file_name, std::ios::in | std::ios::binary);
	//osgDB::ifstream ifs;
	//ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);
	if (ifs.fail())
		return -1;
	liblas::Reader* reader = 0;
	unsigned nbOfPoints = 0;

	std::vector<std::string> dimensions;

	try
	{
		reader = new liblas::Reader(liblas::ReaderFactory().CreateWithStream(ifs));
		liblas::Header const& header = reader->GetHeader();
		dimensions = header.GetSchema().GetDimensionNames();
		nbOfPoints = header.GetPointRecordsCount();
		goffset_x = header.GetOffsetX();
		goffset_y = header.GetOffsetY();
		goffset_z = header.GetOffsetZ();
	}
	catch (...)
	{
		delete reader;
		ifs.close();
		return -1;
	}
	if (nbOfPoints == 0)
	{
		delete reader;
		ifs.close();
		return -1;
	}

	cloud_ptr->height = 1;
	cloud_ptr->width = nbOfPoints;
	cloud_ptr->resize(nbOfPoints);

	int index = 0;
	while (reader->ReadNextPoint())
	{
		const liblas::Point & p = reader->GetPoint();

		cloud_ptr->points[index].x = p.GetX() - goffset_x;
		cloud_ptr->points[index].y = p.GetY() - goffset_y;
		cloud_ptr->points[index].z = p.GetZ() - goffset_z;
		++index;
	}
	if (reader)
	{
		delete reader;
		reader = nullptr;
	}
	ifs.close();

	return 0;
}

void WriteLas(std::string filename, pcl::PointCloud<pcl::PointXYZ> &filtered_cloud)
{
	std::ofstream ofs(filename, std::ios::out | std::ios::binary);
	if (ofs.fail())
		return;
	liblas::Writer *writer = 0;
	double scale = 1.0;
	//统计点的个数
	pcl::PointXYZ minP;
	pcl::PointXYZ maxP;
	pcl::getMinMax3D<pcl::PointXYZ>(filtered_cloud, minP, maxP);
	double xx = maxP.x - minP.x;
	double yy = maxP.y - minP.y;
	double zz = maxP.z - minP.z;
	try
	{
		liblas::Header header;
		//LAZ support based on extension!
		/*if (QFileInfo(filename).suffix().toUpper() == "LAZ")
		{
		header.SetCompressed(true);
		}*/
		//header.SetDataFormatId(liblas::ePointFormat3);
		{
			header.SetMin(static_cast<double>(minP.x / scale + goffset_x),
				static_cast<double>(minP.y / scale + goffset_y),
				static_cast<double>(minP.z / scale + goffset_z));
			header.SetMax(static_cast<double>(maxP.x / scale + goffset_x),
				static_cast<double>(maxP.y / scale + goffset_y),
				static_cast<double>(maxP.z / scale + goffset_z));
			//Set offset & scale, as points will be stored as boost::int32_t values (between 0 and 4294967296)
			//int_value = (double_value-offset)/scale
			header.SetOffset(static_cast<double>(minP.x + goffset_x), static_cast<double>(minP.y + goffset_y), static_cast<double>(minP.z + goffset_z));

			header.SetScale(1.0e-9 * std::max<double>(xx, 1.0e-8), //result must fit in 32bits?!
				1.0e-9 * std::max<double>(yy, 1.0e-8),
				1.0e-9 * std::max<double>(zz, 1.0e-8));
		}
		int m_ptnum = filtered_cloud.size();

		char *old_locale = _strdup(setlocale(LC_CTYPE, NULL));
		header.SetPointRecordsCount(m_ptnum);

		//DGM FIXME: doesn't seem to do anything;)
		header.SetDataFormatId(liblas::ePointFormat0);
		//if (!loadIten) //we must remove the colors dimensions!
		//{
		//	liblas::Schema schema = header.GetSchema();
		//	boost::optional< liblas::Dimension const& > InteDim = schema.GetDimension("Intensity");
		//	if (InteDim)
		//		schema.RemoveDimension(InteDim.get());
		//	header.SetSchema(schema);
		//}
		//header.SetDataFormatId(liblas::ePointFormat2);

		writer = new liblas::Writer(ofs, header);
	}
	catch (...)
	{
		return;
	}
	liblas::Point point(&writer->GetHeader());
	for (int i = 0; i < filtered_cloud.size(); i++)
	{
		double x = static_cast<double>(filtered_cloud.points[i].x + goffset_x);
		double y = static_cast<double>(filtered_cloud.points[i].y + goffset_y);
		double z = static_cast<double>(filtered_cloud.points[i].z + goffset_z);
		point.SetCoordinates(x, y, z);
		//point.SetColor(liblas::Color(cloud->points[i].r << 8, cloud->points[i].g << 8, cloud->points[i].b << 8)); //DGM: LAS colors are stored on 16 bits!
		writer->WritePoint(point);
	}
	delete writer;
	ofs.close();
	return;
}

int main(int argc, char** argv)
{
	gflags::ParseCommandLineFlags(&argc,&argv,true);
	std::string las_path = FLAGS_las_path;
	std::string output_dir = FLAGS_output_dir;
	double output_res = FLAGS_output_res;
	int output_dep = FLAGS_output_dep;

	if ("" == las_path)
	{
		std::cout << "point cloud file path can not be empty."<< std::endl;
		return 0;
	}

	if ("" == output_dir)
	{
		std::cout << "output dir can not be empty." << std::endl;
		return 0;
	}

	std::replace(las_path.begin(), las_path.end(), '\\', '/');
	int pos = las_path.find_last_of('/');
	std::string las_name = las_path.substr(pos + 1);
	pos = las_name.find_last_of('.');
	las_name = las_name.substr(0, pos);

	std::replace(output_dir.begin(), output_dir.end(), '\\', '/');
	if ('/' != output_dir[output_dir.size() - 1])
	{
		output_dir = output_dir + "/";
	}

	using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
	CloudXYZ::Ptr cloud_ptr(new CloudXYZ());
	ReadLas(las_path, cloud_ptr);

	//初始化Octree
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(output_res);
	octree.setInputCloud(cloud_ptr);
	octree.defineBoundingBox();
	octree.addPointsFromInputCloud();
	
	pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> vc_octree(output_res);
	//create octree structure
	vc_octree.setInputCloud(cloud_ptr);
	//update bounding box automatically
	vc_octree.defineBoundingBox();
	//add points in the tree
	vc_octree.addPointsFromInputCloud();

	pcl::PointXYZ min_pt;
	pcl::PointXYZ max_pt;
	
	pcl::getMinMax3D(*cloud_ptr, min_pt, max_pt);

	VoxelCenter voxel_center(max_pt, min_pt, vc_octree);

	for (int depth = 0; depth < output_dep; ++depth)
	{
		voxel_center.SetDepth(depth);

		//计算中心点
		CloudXYZ::Ptr center_point_ptr(new CloudXYZ());
		if (voxel_center.Validate())
		{
			center_point_ptr = voxel_center.GetVoxelCenter();
		}
		else
		{
			std::cout << "the voxel size is too big in depth: "<<std::to_string(depth)<< std::endl;
			break;
		}

		CloudXYZ filtered_cloud;
		filtered_cloud.reserve(center_point_ptr->size());

		//采样
		std::cout << "depth: " << std::to_string(depth) << std::endl;
#pragma omp parallel for
		for (int i = 0; i < center_point_ptr->size(); ++i)
		{
			auto cen_p = center_point_ptr->points[i];
			std::vector<int> pointIdxNKNSearch;
			std::vector<float> pointNKNSquaredDistance;
			if (octree.nearestKSearch(cen_p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				auto p = cloud_ptr->points[pointIdxNKNSearch[0]];
				if (voxel_center.IsInVoxel(cen_p, p))
				{
					filtered_cloud.push_back(p);
				}
			}
		}
		std::string output_str = output_dir + las_name + "_d" + std::to_string(depth + 1) + ".las";
		WriteLas(output_str, filtered_cloud);
	}

	CopyFile(las_path.c_str(), (output_dir + las_name + "_d0.las").c_str(), true);

	return 0;
}