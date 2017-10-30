#pragma once
#include <pcl\point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>

class VoxelCenter
{
public:
	VoxelCenter(pcl::PointXYZ max_pt, pcl::PointXYZ min_pt);
	~VoxelCenter();
	void SetDepth(int depth);
	void SetResoluton(float resolution);
	bool Validate();
	bool IsInVoxel(pcl::PointXYZ &cen_p, pcl::PointXYZ &p);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  GetVoxelCenter();

private:
	pcl::PointXYZ max_pt_;
	pcl::PointXYZ min_pt_;
	float max_l_;
	float max_w_;
	float max_h_;
	int depth_;
	float resolution_;
};
