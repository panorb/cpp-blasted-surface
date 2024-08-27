#pragma once

#include "segmenter_utility.hpp"

/// \class BoundaryVolumeHierarchy
///
/// \brief Breadth-first octree data structure
///
/// BoundaryVolumeHierarchy is different than Octree because it partitions
/// space on-demand in a breadth-first fashion rather than all at once in
/// depth-first order. Instead of specifying max_depth, this BVH can stop
/// partitioning once a node has less than min_points associated with it.
/// These features make BoundaryVolumeHierarchy more amenable to efficiently
/// detecting planes in hierarchical subregions of the point cloud.
class BoundaryVolumeHierarchy {
public:
    static constexpr int DIMENSION = 3;
    static constexpr size_t NUM_CHILDREN = 8;

    /// \brief Constructor for the root node of the octree.
    ///
    /// \param point_cloud is the associated set of points being partitioned
    BoundaryVolumeHierarchy(const SPointCloud* point_cloud,
        const SNormalCloud* normal_cloud,
        const Eigen::Vector3f& min_bound,
        const Eigen::Vector3f& max_bound,
        size_t min_points = 1,
        float min_size = 0.0)
        : point_cloud_(point_cloud),
        normal_cloud_(normal_cloud),
        min_points_(min_points),
        min_size_(min_size),
        leaf_(true),
        level_(0),
        child_index_(0) {
        // set origin of root node and size of each child node (cubes)
        center_ = (min_bound + max_bound) / 2;
        size_ = (max_bound - min_bound).maxCoeff();

        // since this is the root, all the point cloud's indices are contained
        indices_ = std::vector<size_t>(point_cloud->points.size());
        std::iota(indices_.begin(), indices_.end(), 0);
    }

    /// \brief Partition a leaf node's points into NUM_CHILDREN subdivisions
    void Partition();

public:
    const SPointCloud* point_cloud_;
    const SNormalCloud* normal_cloud_;
    std::array<std::shared_ptr<BoundaryVolumeHierarchy>, NUM_CHILDREN>
        children_;
    Eigen::Vector3f center_;
    size_t min_points_;
    float min_size_;
    float size_;
    bool leaf_;
    size_t level_;
    size_t child_index_;
    std::vector<size_t> indices_;

private:
    /// \brief Private constructor for creating children.
    ///
    /// \param point_cloud is the (original) set of points being partitioned
    /// \param level in tree that this child lives on
    /// \param center coordinate of this child node
    /// \param size of this child (same for all nodes at this level)
    BoundaryVolumeHierarchy(const SPointCloud* point_cloud,
        const SNormalCloud* normal_cloud,
        size_t level,
        const Eigen::Vector3f& center,
        size_t min_points,
        float min_size,
        float size,
        size_t child_index);

    /// \brief Calculate the center coordinate of a child node.
    ///
    /// For a root node with center_ == (0, 0, 0) and size_ == 2,
    /// child_size == 1 and:
    ///   child_index 0: center == (-0.5, -0.5, -0.5)
    ///   child_index 1: center == (-0.5, -0.5,  0.5)
    ///   child_index 2: center == (-0.5,  0.5, -0.5)
    ///   child_index 3: center == (-0.5,  0.5,  0.5)
    ///   child_index 4: center == ( 0.5, -0.5, -0.5)
    ///   child_index 5: center == ( 0.5, -0.5,  0.5)
    ///   child_index 6: center == ( 0.5,  0.5, -0.5)
    ///   child_index 7: center == ( 0.5,  0.5,  0.5)
    ///
    /// \param child_index indicates which child
    /// \param child_size of the child's bounding volume (cube)
    Eigen::Vector3f CalculateChildCenter(size_t child_index,
        float child_size) const;

    /// \brief Calculate child index given a position
    ///
    /// \param position of point to find child index of
    size_t CalculateChildIndex(const Eigen::Vector3f& position) const;
};

using BoundaryVolumeHierarchyPtr = std::shared_ptr<BoundaryVolumeHierarchy>;