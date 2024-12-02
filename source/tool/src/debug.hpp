#pragma once

#include <vector>
#include "blast/point_cloud.hpp"

namespace blast
{
	std::vector<size_t> get_k_highest_points(blast::Point_cloud& cloud, int k);
	std::vector<size_t> get_k_lowest_points(Point_cloud& cloud, int k);
}

