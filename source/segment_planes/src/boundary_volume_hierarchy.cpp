#include "blast/boundary_volume_hierarchy.hpp"


/// \brief Partition a leaf node's points into NUM_CHILDREN subdivisions
void BoundaryVolumeHierarchy::Partition() {
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
            CalculateChildIndex(point_cloud_->points[pidx].getVector3fMap());
        // if child does not yet exist, create and initialize
        if (children_[cidx] == nullptr) {
            const Eigen::Vector3f child_center =
                CalculateChildCenter(cidx, child_size);
            children_[cidx].reset(new BoundaryVolumeHierarchy(
                point_cloud_, normal_cloud_, level_ + 1, child_center, min_points_,
                min_size_, child_size, cidx));
            children_[cidx]->indices_.reserve(indices_.size());
        }
        children_[cidx]->indices_.push_back(pidx);
    }

    // now that I have children, I am no longer a leaf node
    leaf_ = false;
}

/// \brief Private constructor for creating children.
///
/// \param point_cloud is the (original) set of points being partitioned
/// \param level in tree that this child lives on
/// \param center coordinate of this child node
/// \param size of this child (same for all nodes at this level)

BoundaryVolumeHierarchy::BoundaryVolumeHierarchy(const SPointCloud* point_cloud, const SNormalCloud* normal_cloud, size_t level, const Eigen::Vector3f& center, size_t min_points, float min_size, float size, size_t child_index)
    : point_cloud_(point_cloud),
    normal_cloud_(normal_cloud),
    center_(center),
    min_points_(min_points),
    min_size_(min_size),
    size_(size),
    leaf_(true),
    level_(level),
    child_index_(child_index) {}

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

Eigen::Vector3f BoundaryVolumeHierarchy::CalculateChildCenter(size_t child_index, float child_size) const {
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

/// \brief Calculate child index given a position
///
/// \param position of point to find child index of

size_t BoundaryVolumeHierarchy::CalculateChildIndex(const Eigen::Vector3f& position) const {
    size_t child_index = 0;
    for (size_t d = 0; d < DIMENSION; d++) {
        child_index |= (position(d) > center_(d)) << (DIMENSION - d - 1);
    }
    return child_index;
}
