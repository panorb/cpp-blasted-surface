#pragma once

#include "Eigen/Core"
#include <blast/planes/bounding_box.hpp>

#include "blast/detected_plane_segment.hpp"

class Grid_sampler
{
	Grid_sampler(std::shared_ptr<Oriented_bounding_box> plane);
};

std::vector<Eigen::Vector3f> sample_points_on_grid(Detected_plane_segment& plane, float grid_size, float distance_along_normal);