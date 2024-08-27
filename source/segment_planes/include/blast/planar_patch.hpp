#include "segmenter_utility.hpp"

/// \brief Planar patch container
struct PlanarPatch {
    float GetSignedDistanceToPoint(const Eigen::Vector3f& point) const;

    Eigen::Vector3f center_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal_ = Eigen::Vector3f::Zero();
    float dist_from_origin_ = 0;
};
