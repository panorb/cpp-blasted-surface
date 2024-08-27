#pragma once

#include <pcl/octree/octree.h>
#include <pcl/common/common_headers.h>

class OctreeDepthIndexableSearch : public pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> {
public:
	/** \brief Constructor.
	* \param[in] resolution octree resolution at lowest octree level
	 */
	OctreeDepthIndexableSearch(const double resolution)
		: pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution)
	{}

	void depthNodeSearchRecursive(const pcl::octree::OctreeKey& key_arg, const pcl::uindex_t start_depth, pcl::uindex_t depth_mask_arg, pcl::octree::OctreeNode* node_arg, pcl::Indices& result_arg);
	pcl::uindex_t depthNodeSearch(pcl::uindex_t index, pcl::uindex_t start_depth, pcl::Indices& point_idx_data);
};
	