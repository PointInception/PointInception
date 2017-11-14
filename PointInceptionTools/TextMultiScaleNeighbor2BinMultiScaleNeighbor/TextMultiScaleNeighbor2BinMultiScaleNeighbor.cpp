// TextMultiScaleNeighbor2BinMultiScaleNeighbor.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

/*
以文本格式的邻域点云为输入，转换为二进制格式
*/
int main(int argc,char ** argv)
{
	if (argc != 2) {
		std::cout << "参数设置错误，请以需要转换的文本格式数据文件名为参数\n";
		return 0;
	}
	std::string filename = argv[1];
	std::ifstream ifs;
	ifs.open(filename);
	if (!ifs.good()) {
		std::cout << "无法打开输入文件\n";
		return 0;
	}
	//读取一行，获得尺度数以及各尺度的临近点个数
	int num_scale = 0;
	int num_neighbor = 0;
	std::string strline;
	while (getline(ifs,strline)){
		if (strline.empty())
			continue;
		strline = strline.substr(strline.find_first_not_of(' '));
		if (strline.empty())
			continue;
		strline = strline.substr(strline.find_first_of(':')+1);
		std::string neighbor_points = strline.substr(0, strline.find_first_of(';'));
		//在neighbor_points中统计逗号的个数，邻域点的个数为逗号个数+1
		while (neighbor_points.find_first_of(',') != std::string::npos){
			num_neighbor++;
			neighbor_points = neighbor_points.substr(neighbor_points.find_first_of(',') + 1);
		}
		//邻域点的个数为逗号个数+1
		num_neighbor++;
		while (strline.find_first_of(';') != std::string::npos){
			num_scale++;
			strline = strline.substr(strline.find_first_of(';') + 1);
		}
		break;
	}
	//将文件指针跳转到开始位置
	ifs.seekg(0, std::ios::beg);
	//输出文件名
	std::string bin_filename = filename.substr(0, filename.find_last_of('.'));
	bin_filename += ".bin";
	//说明文件名
	std::string meta_filename = filename.substr(0, filename.find_last_of('.'));
	meta_filename += ".meta";
	std::ofstream meta_ofs(meta_filename);
	meta_ofs << "#尺度个数\n";
	meta_ofs << "ScaleNum:" << num_scale << "\n";
	meta_ofs << "#每个尺度的点云邻域个数\n";
	meta_ofs << "NeighborNum:" << num_neighbor << "\n";
	meta_ofs.close();
	std::ofstream bin_ofs;
	bin_ofs.open(bin_filename, std::ios::binary);
	while (getline(ifs, strline)) {
		std::istringstream istr(strline);
		float x, y, z;
		int label;
		istr >> x >> y >> z >> label;
		bin_ofs.write((const char *)&x, sizeof(float));
		bin_ofs.write((const char *)&y, sizeof(float));
		bin_ofs.write((const char *)&z, sizeof(float));
		bin_ofs.write((const char *)&label, sizeof(int));
		//冒号分割符
		char seperator;
		istr >> seperator;
		for (int iScale = 0; iScale < num_scale; ++iScale) {
			for (int jNeighbor = 0; jNeighbor < num_neighbor; ++jNeighbor) {
				istr >> x >> y >> z;
				bin_ofs.write((const char *)&x, sizeof(float));
				bin_ofs.write((const char *)&y, sizeof(float));
				bin_ofs.write((const char *)&z, sizeof(float));
				//点与点之间的逗号分隔符
				//尺度与尺度之间的分号分隔符
				istr >> seperator;
			}
		}
	}
	ifs.close();
	bin_ofs.close();
	std::cout << filename << " " << "转换完成\n";
    return 0;
}

