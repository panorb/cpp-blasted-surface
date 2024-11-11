#pragma once

#include "Eigen/Core"
#include <blast/planes/bounding_box.hpp>

#include "blast/detected_plane_segment.hpp"

class Grid_sampler
{
public:
	Grid_sampler() {};


	float grid_size_ = 10.0f;
	float angle_bias_ = 5.0f;
	float point_support_radius_ = 3.0f;

	void render_controls();
	std::vector<Eigen::Vector3f> sample(Detected_plane_segment& plane, const std::vector<Eigen::Vector3f>& cloud_points);
};
