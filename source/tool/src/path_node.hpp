#pragma once

#include <Eigen/Core>

namespace blast
{
	struct Path_node
	{
		Eigen::Vector3f location;
		Eigen::Vector3f lookat;
		Eigen::Vector3f x_unit;
		Eigen::Vector3f y_unit;
	};
}