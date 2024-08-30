#include "blast/planes/octree_depth_indexable.hpp"

void Octree_depth_indexable_search::depth_node_search_recursive(const pcl::octree::OctreeKey& key_arg, const pcl::uindex_t start_depth, pcl::uindex_t depth_mask_arg, pcl::octree::OctreeNode* node_arg, pcl::Indices& result_arg)
{
	if (node_arg->getNodeType() == pcl::octree::BRANCH_NODE) {
		auto* branch_node = static_cast<BranchNode*>(node_arg);

		if (depth_mask_arg <= start_depth) {
			for (int i = 0; i < 8; i++) {
				pcl::octree::OctreeNode* child_node = branch_node->getChildPtr(i);
				if (child_node) {
					depth_node_search_recursive(key_arg, start_depth, depth_mask_arg >> 1, child_node, result_arg);
				}
			}
		}
		else {
			// find branch child from key
			unsigned char child_idx = key_arg.getChildIdxWithDepthMask(depth_mask_arg);

			pcl::octree::OctreeNode* child_node = branch_node->getChildPtr(child_idx);

			if (child_node)
				depth_node_search_recursive(key_arg, start_depth, depth_mask_arg >> 1, child_node, result_arg);
		}
	}
	else if (node_arg->getNodeType() == pcl::octree::LEAF_NODE) {
		auto* leaf_node = static_cast<LeafNode*>(node_arg);
		for (auto element : leaf_node->getContainerPtr()->getPointIndicesVector()) {
			result_arg.push_back(element);
		}
		return;
	}

}

pcl::uindex_t Octree_depth_indexable_search::depth_node_search(pcl::uindex_t index, pcl::uindex_t start_depth, pcl::Indices& point_idx_data)
{
	auto* r = root_node_;
	pcl::PointXYZ p = getPointByIndex(index);

	pcl::octree::OctreeKey key;
	genOctreeKeyforPoint(p, key);

	depth_node_search_recursive(key, start_depth, depth_mask_, r, point_idx_data);
	return 0;
}
