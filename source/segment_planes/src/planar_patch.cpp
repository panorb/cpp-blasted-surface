#include "blast/planar_patch.hpp"

float PlanarPatch::GetSignedDistanceToPoint(const Eigen::Vector3f& point) const {
    return normal_.dot(point) + dist_from_origin_;
}

