#include "segmenter_utility.hpp"

struct Planar_patch {
    float get_signed_distance_to_point(const Eigen::Vector3f& point) const;

    Eigen::Vector3f center_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal_ = Eigen::Vector3f::Zero();
    float dist_from_origin_ = 0;
};
