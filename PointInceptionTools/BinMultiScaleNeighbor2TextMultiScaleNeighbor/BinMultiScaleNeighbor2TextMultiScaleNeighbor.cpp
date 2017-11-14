// BinMultiScaleNeighbor2TextMultiScaleNeighbor.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <vector>
/*
�Զ����Ƹ�ʽ���������Ϊ���룬ת��Ϊ�ı���ʽ
*/
int main(int argc,char ** argv)
{
	if (argc != 2) {
		std::cout << "�Զ����Ƹ�ʽ�������(*.bin)Ϊ���������˵���ļ�(*.meta)Ϊ����\n";
		return 0;
	}
	std::string filename = argv[1];
	if (filename.find_last_of('.') == std::string::npos) {
		std::cout << "�����ļ�û�к�׺��\n";
		return 0;
	}
	std::string ext = filename.substr(filename.find_last_of('.'));
	std::string meta_filename;
	std::string bin_filename;
	if (ext == ".bin") {
		bin_filename = filename;
		meta_filename = filename.substr(0, filename.find_last_of('.'));
		meta_filename += ".meta";
	}
	else if (ext == ".meta") {
		meta_filename = filename;
		bin_filename = filename.substr(0, filename.find_last_of('.'));
		bin_filename += ".bin";
	}
	else {
		std::cout << "�����ļ��Ȳ��Ƕ����������ļ�(*.bin)Ҳ����˵���ļ�(*.meta)\n";
		return 0;
	}
	std::tr2::sys::path bin_path(bin_filename);
	std::tr2::sys::path meta_path(meta_filename);
	if (!std::tr2::sys::exists(bin_filename)) {
		std::cout << "�����������ļ�������\n";
		return 0;
	}
	if (!std::tr2::sys::exists(meta_path)) {
		std::cout << "˵���ļ�������\n";
		return 0;
	}
	int num_scale = 0;
	int num_neighbor = 0;
	std::ifstream meta_ifs(meta_filename);
	if (!meta_ifs.good()) {
		std::cout << "�޷���˵���ļ�\n";
		return 0;
	}
	std::string strline;
	while (getline(meta_ifs,strline))
	{
		strline = strline.substr(strline.find_first_not_of(' '));
		if (strline.empty())
			continue;
		if (strline[0] == '#')
			continue;
		if (strline.find(':') != std::string::npos) {
			std::string name = strline.substr(0, strline.find(':'));
			if (name == "ScaleNum") {
				std::istringstream istr(strline.substr(strline.find(':') + 1));
				istr >> num_scale;
			}
			else if (name == "NeighborNum") {
				std::istringstream istr(strline.substr(strline.find(':') + 1));
				istr >> num_neighbor;
			}
		}
	}
	meta_ifs.close();
	if (num_neighbor < 1 || num_scale < 1) {
		std::cout << "������ĳ߶������ߵ����������\n";
		std::cout << "NumScale:" << num_scale << "\n";
		std::cout << "NumNeighbor:" << num_neighbor << "\n";
		return 0;
	}
	std::string txt_filename = bin_filename.substr(0, bin_filename.find_last_of('.'));
	txt_filename += ".txt";
	std::ofstream ofs(txt_filename);

	std::ifstream bin_ifs(bin_filename, std::ios::binary);

	while (true)
	{
		float x, y, z;
		int label;

		std::vector<float> neighbor(3 * num_neighbor, 0.0f);
		std::vector<std::vector<float>> neighbors(num_scale, neighbor);

		bin_ifs.read((char *)&x, sizeof(float));
		bin_ifs.read((char *)&y, sizeof(float));
		bin_ifs.read((char *)&z, sizeof(float));
		bin_ifs.read((char *)&label, sizeof(int));

		for (int iScale = 0; iScale < num_scale; ++iScale) {
			float tx, ty, tz;
			for (int jNeighbor = 0; jNeighbor < num_neighbor; ++jNeighbor) {
				bin_ifs.read((char *)&tx, sizeof(float));
				bin_ifs.read((char *)&ty, sizeof(float));
				bin_ifs.read((char *)&tz, sizeof(float));
				neighbors[iScale][3 * jNeighbor] = tx;
				neighbors[iScale][3 * jNeighbor + 1] = ty;
				neighbors[iScale][3 * jNeighbor + 2] = tz;
			}
		}
		if (bin_ifs.eof()) {
			break;
		}
		ofs << x << " " << y << " " << z << " " << label << ":";
		for (int iScale = 0; iScale < num_scale; ++iScale) {
			for (int jNeighbor = 0; jNeighbor < num_neighbor; ++jNeighbor) {
				ofs << neighbors[iScale][3 * jNeighbor] << " "
					<< neighbors[iScale][3 * jNeighbor + 1] << " "
					<< neighbors[iScale][3 * jNeighbor + 2];
				if (jNeighbor < num_neighbor - 1) {
					ofs << ",";
				}				
			}
			ofs << ";";
		}
		ofs << "\n";
	}
	ofs.close();
	bin_ifs.close();
	std::cout <<bin_filename<<":"<< "ת�����\n";
	return	0;
}

