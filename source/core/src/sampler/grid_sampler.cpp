#include "blast/sampler/grid_sampler.hpp"

#include <imgui.h>


void Grid_sampler::render_controls()
{
	ImGui::DragFloat("Grid size", &grid_size_, .5, 1.0, 30.0);
	ImGui::DragFloat("Angle bias", &angle_bias_,	1.0, 1.0, 50.0);
	ImGui::DragFloat("Point support radius", &point_support_radius_, .3, 0.0, 40.0);
}

std::vector<Eigen::Vector3f> Grid_sampler::sample(Oriented_bounding_box& bbox, const std::vector<Eigen::Vector3f>& cloud_points)
{
	Eigen::Vector3f normal = (bbox.R_* Eigen::Vector3f(0, 0, 1)).normalized();

	normal.normalize();

	Eigen::Vector3f x_axis = bbox.R_ * Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f y_axis = bbox.R_ * Eigen::Vector3f(0, 1, 0);

	Eigen::Vector3f origin = bbox.get_center() - 0.5 * bbox.extent_(0) * x_axis - 0.5 * bbox.extent_(1) * y_axis;

	Eigen::Vector3f x_axis_unit = x_axis.normalized();
	Eigen::Vector3f y_axis_unit = y_axis.normalized();

	float x_axis_length = bbox.extent_(0);
	float y_axis_length = bbox.extent_(1);

	int x_axis_num = x_axis_length / grid_size_;
	int y_axis_num = y_axis_length / grid_size_;


	std::vector<Eigen::Vector3f> sample_points;

	for (int i = 0; i < x_axis_num; i++)
	{
		for (int j = 0; j < y_axis_num; j++)
		{
			Eigen::Vector3f point = origin + i * grid_size_ * x_axis_unit + j * grid_size_ * y_axis_unit;

			for (auto& pt : cloud_points)
			{
				// check if point is near pt
				if ((pt - point).norm() <= point_support_radius_)
				{
					// point is supported and should be added.
					sample_points.push_back(point);
					spdlog::info("Proposed point (grid: {}, {}) is supported. Adding...", i, j);
					break;
				}
			}
		}
	}

	return sample_points;
}
