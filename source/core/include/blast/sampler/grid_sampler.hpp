#pragma once

#include "Eigen/Core"
#include <blast/planes/bounding_box.hpp>

class Grid_sampler
{
	Grid_sampler(std::shared_ptr<Oriented_bounding_box> plane);
};

std::vector<Eigen::Vector3f> sample_points_on_grid(std::shared_ptr<Oriented_bounding_box> plane, float grid_size);