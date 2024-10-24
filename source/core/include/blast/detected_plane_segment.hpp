#pragma once
#include <random>

#include "planes/bounding_box.hpp"

struct Detected_plane_segment
{
	std::shared_ptr<Oriented_bounding_box> bbox;
	Eigen::Vector3f normal;
	std::vector<Eigen::Vector3f> sample_points;
	bool selected = true;

	Detected_plane_segment(std::shared_ptr<Oriented_bounding_box> bbox, Eigen::Vector3f normal)
		: bbox(bbox), normal(normal)
	{
		this->uuid = generate_uuid();
	}

	bool intersect_ray(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_direction, float& intersection_distance)
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

		// TODO: Calculate distance from ray_origin to intersection
		if (tmin <= tmax)
		{
			// Calculate intersection point
			intersection_distance = tmin;
			return true;
		}

		return false;
	}

	const std::string& get_uuid()
	{
		return uuid;
	}

private:
	std::string uuid;
	std::string generate_uuid()
	{
		static std::random_device dev;
		static std::mt19937 rng(dev());

		std::uniform_int_distribution<int> dist(0, 15);

		const char* v = "0123456789abcdef";
		const bool dash[] = { 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0 };

		std::string res;
		for (int i = 0; i < 16; i++) {
			if (dash[i]) res += "-";
			res += v[dist(rng)];
			res += v[dist(rng)];
		}
		return res;
	}
};
