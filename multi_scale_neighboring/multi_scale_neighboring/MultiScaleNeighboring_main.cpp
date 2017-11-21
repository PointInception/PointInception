#include "io.h"
#include <time.h>
#include  <iostream>
#include <vector>
#include <Windows.h>
#include <fstream>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/search/kdtree.h"
#include "liblas/liblas.hpp"
#include <pcl/common/io.h>
#include <gflags\gflags.h>

DEFINE_string(las_name, "", "原始点云文件");
DEFINE_string(scale_folder,"","多尺度点云所在文件夹");
DEFINE_string(label_folder, "", "标签文件所在文件夹");
DEFINE_int32(nsize, 20, "最近邻点的数量");
DEFINE_string(file_flag, "", "输入点云类型:train_file或者test_file");

using namespace std;
using namespace pcl;
struct Pointneighbor //only 1 and 0
{
	pcl::PointXYZ centerp;
	int label;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scaleneighbor;
};

std::string wstring2string(std::wstring wstr)
{
	std::string result;
	int len = WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), NULL, 0, NULL, NULL);
	if (len <= 0)return result;
	char* buffer = new char[len + 1];
	if (buffer == NULL)return result;
	WideCharToMultiByte(CP_ACP, 0, wstr.c_str(), wstr.size(), buffer, len, NULL, NULL);
	buffer[len] = '\0';
	result.append(buffer);
	delete[] buffer;
	return result;
}

std::vector<std::string> getfilelist(std::string las_name, std::string &file_folder)
{
	int index = las_name.rfind('\\');
	int index2 = las_name.rfind('.');
	std::string filename = las_name.substr(index+1, index2-index-1);
	std::vector<std::string> file_list;
	intptr_t hFile = 0;
	struct _finddata_t fileInfo;
	std::string filter = file_folder +"\\"+ filename+"_d*.las";
	if ((hFile = _findfirst(filter.c_str(), &fileInfo)) == -1) {
		std::cout << "在目标文件夹中没有找到点云文件..." << std::endl;
		return file_list;
	}
	do {
		if (!(fileInfo.attrib&_A_SUBDIR)) {
			std::string name = fileInfo.name;
			std::string absolute_name = file_folder + "\\" + name;
			file_list.push_back(absolute_name);
		}
	} while (_findnext(hFile, &fileInfo) == 0);
	return file_list;
}

std::string getfile(std::string las_name, std::string &file_folder)
{
	std::string label_name;
	int index = las_name.rfind('\\');
	int index2 = las_name.rfind('.');
	std::string filename = las_name.substr(index + 1, index2 - index - 1);
	std::vector<std::string> file_list;
	intptr_t hFile = 0;
	struct _finddata_t fileInfo;
	std::string filter2 = file_folder + "\\" + filename + ".labels";
	if ((hFile = _findfirst(filter2.c_str(), &fileInfo)) == -1) {
		std::cout << "在目标文件夹中没有找到类别文件..." << std::endl;
		return label_name;
	}
	do {
		if (!(fileInfo.attrib&_A_SUBDIR)) {
			std::string name = fileInfo.name;
			std::string absolute_name = file_folder + "\\" + name;
			label_name = absolute_name;
		}
	} while (_findnext(hFile, &fileInfo) == 0);
	return label_name;
}

std::vector<std::string> select_las(std::vector<std::string> file_list)
{
	int count = 0;
	for (int i = 0; i < file_list.size(); i++)
	{
		int startpos = file_list[i].rfind('_');
		if (file_list[i][startpos +1] == 'd')
		{
			count++;
		}
	}
	std::vector<std::string> select_list;
	select_list.resize(count);
	for (int i = 0; i < file_list.size(); i++)
	{
		int startpos = file_list[i].rfind('_');
		int endpos = file_list[i].rfind('.');
		if (file_list[i][startpos+1] == 'd')
		{
			string snum = file_list[i].substr(startpos+2, endpos-startpos-2);
			int num = atoi(snum.c_str());
			select_list.at(num) = file_list[i];
		}
	}
	return select_list;
}

void readlas(std::string filename, 
			 pcl::PointCloud<pcl::PointXYZL>::Ptr &origioncloud,
	float &Xmin,
	float &Ymin,
	float&Zmin)
{
	std::fstream fread(filename, std::ios::in | std::ios::binary);
	if (!fread.is_open())
	{
		std::cout << "las 读取错误" << std::endl;
	}
	unsigned nbOfPoints = 0;
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(fread);
	liblas::Header const& header = reader.GetHeader();
	if (fabs(Xmin-0)<0.00001)
	{
		Xmin = header.GetMinX();
	}
	if (fabs(Ymin - 0)<0.00001)
	{
		Ymin = header.GetMinY();
	}
	if (fabs(Zmin - 0)<0.00001)
	{
		Zmin = header.GetMinZ();
	}
	nbOfPoints = header.GetPointRecordsCount();
	origioncloud->resize(nbOfPoints);
	int i_num = 0;
	while (reader.ReadNextPoint())
	{
		const liblas::Point & p = reader.GetPoint();
		origioncloud->points[i_num].x = p.GetX() - Xmin;
		origioncloud->points[i_num].y = p.GetY() - Ymin;
		origioncloud->points[i_num].z = p.GetZ() - Zmin;
		i_num++;
	}
}

void readlas(std::string filename, 
			 pcl::PointCloud<pcl::PointXYZ>::Ptr &origioncloud, 
					float &Xmin,
					float &Ymin,
					float &Zmin)
{
	std::fstream fread(filename, std::ios::in | std::ios::binary);
	if (!fread.is_open())
	{
		std::cout << "las 读取错误" << std::endl;
	}
	unsigned nbOfPoints = 0;
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(fread);
	liblas::Header const& header = reader.GetHeader();
	if (fabs(Xmin - 0)<0.00001)
	{
		Xmin = header.GetMinX();
	}
	if (fabs(Ymin - 0)<0.00001)
	{
		Ymin = header.GetMinY();
	}
	if (fabs(Zmin - 0)<0.00001)
	{
		Zmin = header.GetMinZ();
	}
	nbOfPoints = header.GetPointRecordsCount();
	origioncloud->resize(nbOfPoints);
	int i_num = 0;
	while (reader.ReadNextPoint())
	{
		const liblas::Point & p = reader.GetPoint();
		origioncloud->points[i_num].x = p.GetX() - Xmin;
		origioncloud->points[i_num].y = p.GetY() - Ymin;
		origioncloud->points[i_num].z = p.GetZ() - Zmin;
		i_num++;
	}
}

void readlabel(std::string labelname, std::vector<int> &labels)
{
	fstream fin2(labelname, ios::in);
	if (!fin2)
	{
		cout << "label读取错误" << endl;
	}
	int count = 0;
	while (fin2.peek() != -1)
	{
		if (count>labels.size())
		{
			cout << "label数目大于点云数目" << std::endl;
		}
		int label;
		fin2 >> label;
		labels[count] = label;
		count++;
	}
	if (count!=labels.size()+1)
	{
		cout << "label数目与点云数目不相等！" << std::endl;
	}
	cout << "label读取完毕" << endl;
	fin2.close();
}
void outputXYZ(Pointneighbor &tmppb, std::ofstream &foutb, 
		float Xmin,
		float Ymin,
		float Zmin )
{
	foutb << tmppb.centerp.x + Xmin << " " << tmppb.centerp.y + Ymin << " " << tmppb.centerp.z + Zmin << " " << tmppb.label << ":";

	for (auto it_1 : tmppb.scaleneighbor)
	{
		int flag = 0;
		for (auto it_2 : *it_1)
		{
			if (flag == it_1->size() - 1)
			{
				foutb << it_2.x + Xmin
					<< " " << it_2.y + Ymin
					<< " " << it_2.z + Zmin;
			}
			else
			{
				foutb << it_2.x + Xmin
					<< " " << it_2.y + Ymin
					<< " " << it_2.z + Zmin << ",";
			}
			flag++;
		}

		foutb << ";";
	}
}
void outputXYZbin(Pointneighbor &tmppb, std::ofstream &foutb,
	float Xmin,
	float Ymin,
	float Zmin)
{
	float x, y, z;
	int label;
	x = tmppb.centerp.x + Xmin;
	y = tmppb.centerp.y + Ymin;
	z = tmppb.centerp.z + Zmin;
	label = tmppb.label;
	foutb.write((const char *)&x, sizeof(float));
	foutb.write((const char *)&y, sizeof(float));
	foutb.write((const char *)&z, sizeof(float));
	foutb.write((const char *)&label, sizeof(int));
	for (auto it_1 : tmppb.scaleneighbor)
	{
		for (auto it_2 : *it_1)
		{
				x = it_2.x + Xmin;
				y = it_2.y + Ymin;
				z = it_2.z + Zmin;
				foutb.write((const char *)&x, sizeof(float));
				foutb.write((const char *)&y, sizeof(float));
				foutb.write((const char *)&z, sizeof(float));
		}

	}
}
void neighbor_calculate(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &scaleclouds,
						int nsize, 
						std::string las_name, 
						float Xmin,
						float Ymin, 
						float Zmin,
						std::vector<int> &labels)
{
	int number_of_file = 0;
	int index = las_name.rfind('.');
	std::string foutfile = las_name.substr(0, index);
	std::ostringstream ostr;
	ostr << number_of_file;
	std::string foutfile_txt = foutfile + "_MultiScaleNeighboring_" + ostr.str()+ ".txt";
	std::ofstream foutb(foutfile_txt, std::ios::out);
	std::cout << "正在构建KdTree" << std::endl;
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> trees;
	trees.resize(scaleclouds.size());
	for (int j = 0; j < trees.size(); j++)
	{
		if (nullptr == trees[j])
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tmptree(new pcl::search::KdTree<PointXYZ>);
			trees[j] = tmptree;
		}
		trees[j]->setInputCloud(scaleclouds[j]);
	}
	std::cout << "KdTree构建完毕 开始选择"<<nsize<<"个领域点" << std::endl;
	for (int j = 0; j < scaleclouds[0]->size(); j++)
	{
		if (labels[j]==0)
		{
			continue;
		}
		Pointneighbor tmppb;
		tmppb.centerp.x = scaleclouds[0]->points[j].x;
		tmppb.centerp.y = scaleclouds[0]->points[j].y;
		tmppb.centerp.z = scaleclouds[0]->points[j].z;
		tmppb.label= labels[j];
		tmppb.scaleneighbor.resize(scaleclouds.size());

		for (int i = 0; i < tmppb.scaleneighbor.size(); i++)
		{
			if (tmppb.scaleneighbor.at(i)==nullptr)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmpc(new pcl::PointCloud<pcl::PointXYZ>);
				tmppb.scaleneighbor.at(i) = tmpc;
			}
			tmppb.scaleneighbor.at(i)->resize(nsize);
		}

		for (int i = 0; i < scaleclouds.size(); i++)
		{
			std::vector<int> pointIdxSearch;
			std::vector<float> pointSquaredDistance;
			trees[i]->nearestKSearch(tmppb.centerp, nsize, pointIdxSearch, pointSquaredDistance);

			pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
			tmpcloud->resize(nsize);
			for (int k = 0; k < pointIdxSearch.size(); k++)
			{
				tmpcloud->points[k] = scaleclouds[i]->points[pointIdxSearch[k]];
			}
			pcl::copyPointCloud(*tmpcloud, *tmppb.scaleneighbor.at(i));
		}
		int64_t wb = foutb.tellp();
		int64_t file_size = (int64_t)1024 * (int64_t)1024 * (int64_t)1024;
		if (wb>file_size)//2GB
		{
			std::cout <<"文件"<< foutfile_txt  << "输出完毕" << std::endl;
			number_of_file++;
			foutb.close();
			std::ostringstream ostr2;
			ostr2 << number_of_file;
			foutfile_txt = foutfile + "_MultiScaleNeighboring_" + ostr2.str() + ".txt";
			foutb.open(foutfile_txt, std::ios::out);
		}
		outputXYZ(tmppb, foutb, Xmin, Ymin, Zmin);

		foutb << std::endl;
	}
}
void neighbor_calculate_by_bin(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &scaleclouds,
	int nsize,
	std::string las_name,
	float Xmin,
	float Ymin,
	float Zmin,
	std::vector<int> &labels)
{
	int number_of_file = 0;
	std::ostringstream ostr;
	ostr << number_of_file;
	int index = las_name.rfind('.');
	std::string meta_filename = las_name.substr(0, index);
	meta_filename =meta_filename+ "_MultiScaleNeighboring_" + ostr.str() +".meta";
	std::ofstream meta_ofs(meta_filename);
	meta_ofs << "#尺度个数\n";
	meta_ofs << "ScaleNum:" << scaleclouds.size() << "\n";
	meta_ofs << "#每个尺度的点云邻域个数\n";
	meta_ofs << "NeighborNum:" << nsize << "\n";
	meta_ofs.close();
	std::string foutfile = las_name.substr(0, index);
	std::string foutfile_txt = foutfile + "_MultiScaleNeighboring_" + ostr.str() + ".bin";
	std::ofstream foutb(foutfile_txt, std::ios::binary);
	std::cout << "正在构建KdTree" << std::endl;
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> trees;
	trees.resize(scaleclouds.size());
#pragma omp parallel for
	for (int j = 0; j < trees.size(); j++)
	{
		if (nullptr == trees[j])
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tmptree(new pcl::search::KdTree<PointXYZ>);
			trees[j] = tmptree;
		}
		trees[j]->setInputCloud(scaleclouds[j]);
	}
	std::cout << "KdTree构建完毕 开始选择" << nsize << "个领域点" << std::endl;
	for (int j = 0; j < scaleclouds[0]->size(); j++)
	{
		if (labels[j] == 0)
		{
			continue;
		}
		Pointneighbor tmppb;
		tmppb.centerp.x = scaleclouds[0]->points[j].x;
		tmppb.centerp.y = scaleclouds[0]->points[j].y;
		tmppb.centerp.z = scaleclouds[0]->points[j].z;
		tmppb.label = labels[j];
		tmppb.scaleneighbor.resize(scaleclouds.size());
#pragma omp parallel for
		for (int i = 0; i < tmppb.scaleneighbor.size(); i++)
		{
			if (tmppb.scaleneighbor.at(i) == nullptr)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmpc(new pcl::PointCloud<pcl::PointXYZ>);
				tmppb.scaleneighbor.at(i) = tmpc;
			}
			tmppb.scaleneighbor.at(i)->resize(nsize);
		}
#pragma omp parallel for
		for (int i = 0; i < scaleclouds.size(); i++)
		{
			std::vector<int> pointIdxSearch;
			std::vector<float> pointSquaredDistance;
			trees[i]->nearestKSearch(tmppb.centerp, nsize, pointIdxSearch, pointSquaredDistance);

			pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud(new pcl::PointCloud<pcl::PointXYZ>);
			tmpcloud->resize(nsize);
			for (int k = 0; k < pointIdxSearch.size(); k++)
			{
				tmpcloud->points[k] = scaleclouds[i]->points[pointIdxSearch[k]];
			}
			pcl::copyPointCloud(*tmpcloud, *tmppb.scaleneighbor.at(i));
		}
		int64_t wb = foutb.tellp();
		int64_t file_size = (int64_t)1024 * (int64_t)1024 * (int64_t)1024;
		if (wb>file_size)//2GB
		{
			std::cout << "文件" << foutfile_txt << "输出完毕" << std::endl;
			number_of_file++;
			foutb.close();
			std::ostringstream ostr2;
			ostr2 << number_of_file;
			foutfile_txt = foutfile + "_MultiScaleNeighboring_" + ostr2.str() + ".bin";
			foutb.open(foutfile_txt, std::ios::binary);
			meta_filename = foutfile + "_MultiScaleNeighboring_" + ostr2.str() + ".meta";
			meta_ofs.open(meta_filename);
			meta_ofs << "#尺度个数\n";
			meta_ofs << "ScaleNum:" << scaleclouds.size() << "\n";
			meta_ofs << "#每个尺度的点云邻域个数\n";
			meta_ofs << "NeighborNum:" << nsize << "\n";
			meta_ofs.close();
		}
		outputXYZbin(tmppb, foutb, Xmin, Ymin, Zmin);
	}
}
int main (int argc, char** argv)
{
	clock_t start_time = clock();
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	std::string las_name = FLAGS_las_name;
	std::string scale_folder = FLAGS_scale_folder;
	std::string label_folder = FLAGS_label_folder;
	std::string file_flag = FLAGS_file_flag;
	int nsize = FLAGS_nsize;

	std::vector<std::string> file_list = getfilelist(las_name, scale_folder);
	std::string label_file;
	if (file_flag== "train_file")
	{
		label_file = getfile(las_name, label_folder);
	}
	else if(file_flag == "test_file")
	{
		std::cout << "正在训练测试样本 类别统一设为0" << std::endl;
	}
	else
	{
		std::cout << " 请输入点云文件类型：train_file或者test_file!" << std::endl;
		return 0;
	}
	std::vector<std::string> select_list = select_las(file_list);
	std::cout << "共有" << select_list.size()<<"个尺度"<< std::endl;
	float Xmin, Ymin, Zmin;
	Xmin = 0.0;
	Ymin = 0.0; 
	Zmin = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr origioncloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "正在读取原始尺度点云" << std::endl;
	readlas(select_list[0], origioncloud,Xmin,Ymin,Zmin);
	std::cout << "正在读取原始尺度类别" << std::endl;
	std::vector<int> labels;
	if (file_flag == "train_file")
	{
		labels.resize(origioncloud->size());
		readlabel(label_file, labels);
	}
	else if (file_flag == "test_file")
	{
		labels.resize(origioncloud->size(),0);
	}
	std::cout << "正在读取多尺度点云" << std::endl;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scaleclouds;
	scaleclouds.resize(select_list.size());
#pragma omp parallel for
	for (int i = 0; i < select_list.size(); i++)
	{
		if (scaleclouds[i]==nullptr)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmpc(new pcl::PointCloud<pcl::PointXYZ>);
			scaleclouds[i]= tmpc;
		}
		if (i==0)
		{
			scaleclouds[i] = origioncloud;
	/*		pcl::copyPointCloud(*origioncloud, *scaleclouds[i]);*/
		}
		else
		{
			readlas(select_list[i], scaleclouds[i], Xmin, Ymin, Zmin);
		}
	}
	std::cout << "多尺度点云读取完毕！" << std::endl;
	//neighbor_calculate(scaleclouds, nsize, las_name, Xmin, Ymin, Zmin,labels);
	neighbor_calculate_by_bin(scaleclouds, nsize, las_name, Xmin, Ymin, Zmin, labels);
	//writepb(allpointn, foutfile, Xmin, Ymin, Zmin);
	clock_t end_time = clock();
	std::cout << "多尺度邻域选择耗时"<<(end_time - start_time) << "s" << std::endl;
    return 0;
}

