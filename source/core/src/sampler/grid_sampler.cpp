#include "blast/sampler/grid_sampler.hpp"

#include "blast/detected_plane_segment.hpp"

std::vector<Eigen::Vector3f> sample_points_on_grid(Detected_plane_segment& plane, float grid_size, float distance_along_normal)
{
	auto bbox = plane.bbox;
	Eigen::Vector3f normal = plane.normal;
	normal.normalize();

	Eigen::Vector3f x_axis = bbox->R_ * Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f y_axis = bbox->R_ * Eigen::Vector3f(0, 1, 0);

	Eigen::Vector3f origin = bbox->get_center() - 0.5 * bbox->extent_(0) * x_axis - 0.5 * bbox->extent_(1) * y_axis;

	Eigen::Vector3f x_axis_unit = x_axis.normalized();
	Eigen::Vector3f y_axis_unit = y_axis.normalized();

	float x_axis_length = bbox->extent_(0);
	float y_axis_length = bbox->extent_(1);

	int x_axis_num = x_axis_length / grid_size;
	int y_axis_num = y_axis_length / grid_size;

	std::vector<Eigen::Vector3f> sample_points;

	for (int i = 0; i < x_axis_num; i++)
	{
		for (int j = 0; j < y_axis_num; j++)
		{
			Eigen::Vector3f point = origin + i * grid_size * x_axis_unit + j * grid_size * y_axis_unit;
			sample_points.push_back(point);
			// bbox->sample_points_.push_back(point);
			// spdlog::info("grid point: {0}, {1}, {2}", point(0), point(1), point(2));
		}
	}

	return sample_points;
}
