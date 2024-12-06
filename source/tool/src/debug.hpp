#pragma once

#include <vector>
#include "blast/point_cloud.hpp"

namespace blast
{
	std::vector<size_t> get_k_highest_points(Point_cloud& cloud, int k, float ignore_distance);
	std::vector<size_t> get_k_lowest_points(Point_cloud& cloud, int k, float ignore_distance);
}

