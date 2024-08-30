#pragma once

#include "segmenter_utility.hpp"

class Boundary_volume_hierarchy {
public:
    static constexpr int DIMENSION = 3;
    static constexpr size_t NUM_CHILDREN = 8;

                Boundary_volume_hierarchy(const SPointCloud* point_cloud,
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

        void partition();

public:
    const SPointCloud* point_cloud_;
    const SNormalCloud* normal_cloud_;
    std::array<std::shared_ptr<Boundary_volume_hierarchy>, NUM_CHILDREN>
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
                            Boundary_volume_hierarchy(const SPointCloud* point_cloud,
        const SNormalCloud* normal_cloud,
        size_t level,
        const Eigen::Vector3f& center,
        size_t min_points,
        float min_size,
        float size,
        size_t child_index);

	///   child_index 0: center == (-0.5, -0.5, -0.5)
    ///   child_index 1: center == (-0.5, -0.5,  0.5)
    ///   child_index 2: center == (-0.5,  0.5, -0.5)
    ///   child_index 3: center == (-0.5,  0.5,  0.5)
    ///   child_index 4: center == ( 0.5, -0.5, -0.5)
    ///   child_index 5: center == ( 0.5, -0.5,  0.5)
    ///   child_index 6: center == ( 0.5,  0.5, -0.5)
    ///   child_index 7: center == ( 0.5,  0.5,  0.5)
                Eigen::Vector3f calculate_child_center(size_t child_index,
        float child_size) const;

                size_t calculate_child_index(const Eigen::Vector3f& position) const;
};

using BoundaryVolumeHierarchyPtr = std::shared_ptr<Boundary_volume_hierarchy>;