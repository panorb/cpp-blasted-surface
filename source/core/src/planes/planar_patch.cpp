#include "blast/planes/planar_patch.hpp"

float Planar_patch::get_signed_distance_to_point(const Eigen::Vector3f& point) const {
    return normal_.dot(point) + dist_from_origin_;
}

