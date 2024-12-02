#pragma once

#include "Eigen/Core"
#include <blast/planes/bounding_box.hpp>


class Grid_sampler
{
public:
	Grid_sampler() {};


	float grid_size_ = 10.0f;
	int circle_radius_ = 5;
	int erosion_iterations_ = 30;
	bool hole_filling_enabled_ = true;

	void render_controls();
	std::vector<Eigen::Vector3f> sample(Oriented_bounding_box& plane, const std::vector<Eigen::Vector3f>& cloud_points);
};
