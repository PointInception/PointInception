// TextMultiScaleNeighbor2BinMultiScaleNeighbor.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

/*
���ı���ʽ���������Ϊ���룬ת��Ϊ�����Ƹ�ʽ
*/
int main(int argc,char ** argv)
{
	if (argc != 2) {
		std::cout << "�������ô���������Ҫת�����ı���ʽ�����ļ���Ϊ����\n";
		return 0;
	}
	std::string filename = argv[1];
	std::ifstream ifs;
	ifs.open(filename);
	if (!ifs.good()) {
		std::cout << "�޷��������ļ�\n";
		return 0;
	}
	//��ȡһ�У���ó߶����Լ����߶ȵ��ٽ������
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
		//��neighbor_points��ͳ�ƶ��ŵĸ����������ĸ���Ϊ���Ÿ���+1
		while (neighbor_points.find_first_of(',') != std::string::npos){
			num_neighbor++;
			neighbor_points = neighbor_points.substr(neighbor_points.find_first_of(',') + 1);
		}
		//�����ĸ���Ϊ���Ÿ���+1
		num_neighbor++;
		while (strline.find_first_of(';') != std::string::npos){
			num_scale++;
			strline = strline.substr(strline.find_first_of(';') + 1);
		}
		break;
	}
	//���ļ�ָ����ת����ʼλ��
	ifs.seekg(0, std::ios::beg);
	//����ļ���
	std::string bin_filename = filename.substr(0, filename.find_last_of('.'));
	bin_filename += ".bin";
	//˵���ļ���
	std::string meta_filename = filename.substr(0, filename.find_last_of('.'));
	meta_filename += ".meta";
	std::ofstream meta_ofs(meta_filename);
	meta_ofs << "#�߶ȸ���\n";
	meta_ofs << "ScaleNum:" << num_scale << "\n";
	meta_ofs << "#ÿ���߶ȵĵ����������\n";
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
		//ð�ŷָ��
		char seperator;
		istr >> seperator;
		for (int iScale = 0; iScale < num_scale; ++iScale) {
			for (int jNeighbor = 0; jNeighbor < num_neighbor; ++jNeighbor) {
				istr >> x >> y >> z;
				bin_ofs.write((const char *)&x, sizeof(float));
				bin_ofs.write((const char *)&y, sizeof(float));
				bin_ofs.write((const char *)&z, sizeof(float));
				//�����֮��Ķ��ŷָ���
				//�߶���߶�֮��ķֺŷָ���
				istr >> seperator;
			}
		}
	}
	ifs.close();
	bin_ofs.close();
	std::cout << filename << " " << "ת�����\n";
    return 0;
}

