#pragma once
#include "planes/bounding_box.hpp"

struct Detected_plane_segment
{
	std::shared_ptr<Oriented_bounding_box> bbox;
	Eigen::Vector3f normal;
	std::vector<Eigen::Vector3f> sample_points;

	Detected_plane_segment(std::shared_ptr<Oriented_bounding_box> bbox, Eigen::Vector3f normal, std::vector<Eigen::Vector3f> sample_points)
		: bbox(bbox), normal(normal), sample_points(sample_points)
	{
	}

	bool intersect_ray(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_direction)
	{
		Eigen::Matrix3f R = bbox->R_;
		Eigen::Vector3f center = bbox->center_;
		Eigen::Vector3f inv_dir = 1.0f / ray_direction.array();
		Eigen::Vector3f extent = bbox->extent_;

		Eigen::Vector3f T = R * (ray_origin - center);

		float tmin = (extent[0] - T[0]) * inv_dir[0];
		float tmax = (extent[0] + T[0]) * inv_dir[0];

		for (int i = 1; i < 3; i++)
		{
			float t1 = (extent[i] - T[i]) * inv_dir[i];
			float t2 = (extent[i] + T[i]) * inv_dir[i];

			tmin = std::max(tmin, std::min(t1, t2));
			tmax = std::min(tmax, std::max(t1, t2));
		}

		if (tmax < 0)
		{
			return false;
		}

		return tmin <= tmax;
	}
};
