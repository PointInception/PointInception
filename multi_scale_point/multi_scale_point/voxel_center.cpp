#include "voxel_center.h"

VoxelCenter::VoxelCenter(pcl::PointXYZ max_pt, pcl::PointXYZ min_pt):
	max_pt_(max_pt),
	min_pt_(min_pt)
{
	max_l_ = max_pt_.x - min_pt_.x;
	max_w_ = max_pt_.y - min_pt_.y;
	max_h_ = max_pt_.z - min_pt_.z;
}

VoxelCenter::~VoxelCenter()
{
}

void VoxelCenter::SetDepth(int depth)
{
	depth_ = depth;
}

void VoxelCenter::SetResoluton(float resolution)
{
	resolution_ = resolution;
}

bool VoxelCenter::Validate()
{
	if (resolution_*std::pow(2, depth_) > max_l_ ||
		resolution_*std::pow(2, depth_) > max_w_ ||
		resolution_*std::pow(2, depth_) > max_h_)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool VoxelCenter::IsInVoxel(pcl::PointXYZ & cen_p, pcl::PointXYZ & p)
{
	float limit = std::pow(2, depth_-1) * resolution_;
	float d_x = abs(cen_p.x - p.x);
	float d_y = abs(cen_p.y - p.y);
	float d_z = abs(cen_p.z - p.z);
	if (d_x <=limit&&d_y <=limit&&d_z <=limit)
	{
		return true;
	}
	else
	{
		return false;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelCenter::GetVoxelCenter()
{
	float true_res = std::pow(2, depth_) * resolution_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr res_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	res_ptr->height = 1;
	int seg_x = std::ceil(max_l_/ true_res);
	int seg_y = std::ceil(max_w_/ true_res);
	int seg_z = std::ceil(max_h_/ true_res);
	res_ptr->width = seg_x*seg_y*seg_z;
	res_ptr->resize(seg_x*seg_y*seg_z);

	pcl::PointXYZ origin;
	origin.x = min_pt_.x + true_res / 2.0;
	origin.y = min_pt_.y + true_res / 2.0;
	origin.z = min_pt_.z + true_res / 2.0;

	int count = 0;
	for (int z = 0; z < seg_z; ++z)
	{
		for (int y = 0; y < seg_y; ++y)
		{
			for (int x = 0; x < seg_x; ++x)
			{
				res_ptr->points[count].x = origin.x + x*true_res;
				res_ptr->points[count].y = origin.y + y*true_res;
				res_ptr->points[count].z = origin.z + z*true_res;
				++count;
			}
		}
	}

	return res_ptr;
}
