#include "voxel_center.h"

VoxelCenter::VoxelCenter(pcl::PointXYZ max_pt, pcl::PointXYZ min_pt,pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> &vc_octree):
	max_pt_(max_pt),
	min_pt_(min_pt),
	vc_octree_(vc_octree)
{
	max_l_ = max_pt_.x - min_pt_.x;
	max_w_ = max_pt_.y - min_pt_.y;
	max_h_ = max_pt_.z - min_pt_.z;
	resolution_ = vc_octree_.getResolution();
}

VoxelCenter::~VoxelCenter()
{
}

void VoxelCenter::SetDepth(int depth)
{
	depth_ = depth;
}

void VoxelCenter::SetOctree(pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>& vc_octree)
{
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
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr res_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	res_ptr->height = 1;
	//res_ptr->width = seg_x*seg_y*seg_z;
	//res_ptr->resize(seg_x*seg_y*seg_z);

	pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it;
	pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Iterator tree_it_end = vc_octree_.end();
	int depth = vc_octree_.getTreeDepth() - depth_;

	int count = 0;
	for (tree_it = vc_octree_.begin(depth); tree_it != tree_it_end; ++tree_it)
	{
		++count;
	}
	res_ptr->width = count;
	res_ptr->points.reserve(count);

	for (tree_it = vc_octree_.begin(depth); tree_it != tree_it_end; ++tree_it)
	{
		Eigen::Vector3f voxel_min, voxel_max;
		vc_octree_.getVoxelBounds(tree_it, voxel_min, voxel_max);
		pcl::PointXYZ pt;
		pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
		pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
		pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
		res_ptr->points.push_back(pt);
	}

	return res_ptr;
}
