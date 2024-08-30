#include "blast/planes/boundary_volume_hierarchy.hpp"


void Boundary_volume_hierarchy::partition() {
    // Nothing to do if already partitioned
    if (!leaf_) return;

    // size of each child
    const float child_size = size_ / 2.;

    // Does this node have enough data to be able to partition further
    if (indices_.size() <= min_points_ || child_size < min_size_ ||
        indices_.size() < 2)
        return;

    // split points and create children
    for (const size_t& pidx : indices_) {
        // calculate child index comparing position to child center
        const size_t cidx =
            calculate_child_index(point_cloud_->points[pidx].getVector3fMap());
        // if child does not yet exist, create and initialize
        if (children_[cidx] == nullptr) {
            const Eigen::Vector3f child_center =
                calculate_child_center(cidx, child_size);
            children_[cidx].reset(new Boundary_volume_hierarchy(
                point_cloud_, normal_cloud_, level_ + 1, child_center, min_points_,
                min_size_, child_size, cidx));
            children_[cidx]->indices_.reserve(indices_.size());
        }
        children_[cidx]->indices_.push_back(pidx);
    }

    // now that I have children, I am no longer a leaf node
    leaf_ = false;
}


Boundary_volume_hierarchy::Boundary_volume_hierarchy(const SPointCloud* point_cloud, const SNormalCloud* normal_cloud, size_t level, const Eigen::Vector3f& center, size_t min_points, float min_size, float size, size_t child_index)
    : point_cloud_(point_cloud),
    normal_cloud_(normal_cloud),
    center_(center),
    min_points_(min_points),
    min_size_(min_size),
    size_(size),
    leaf_(true),
    level_(level),
    child_index_(child_index) {}

///   child_index 0: center == (-0.5, -0.5, -0.5)
///   child_index 1: center == (-0.5, -0.5,  0.5)
///   child_index 2: center == (-0.5,  0.5, -0.5)
///   child_index 3: center == (-0.5,  0.5,  0.5)
///   child_index 4: center == ( 0.5, -0.5, -0.5)
///   child_index 5: center == ( 0.5, -0.5,  0.5)
///   child_index 6: center == ( 0.5,  0.5, -0.5)
///   child_index 7: center == ( 0.5,  0.5,  0.5)

Eigen::Vector3f Boundary_volume_hierarchy::calculate_child_center(size_t child_index, float child_size) const {
    Eigen::Vector3f center;
    for (size_t d = 0; d < DIMENSION; d++) {
        const int signal = (((child_index & (static_cast<uint64_t>(1)
            << (DIMENSION - d - 1))) >>
            (DIMENSION - d - 1))
            << 1) -
            1;
        center(d) = center_(d) + (child_size / 2.) * signal;
    }
    return center;
}


size_t Boundary_volume_hierarchy::calculate_child_index(const Eigen::Vector3f& position) const {
    size_t child_index = 0;
    for (size_t d = 0; d < DIMENSION; d++) {
        child_index |= (position(d) > center_(d)) << (DIMENSION - d - 1);
    }
    return child_index;
}
