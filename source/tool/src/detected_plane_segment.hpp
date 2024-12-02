#pragma once
#include <random>


struct Detected_plane_segment
{
	std::shared_ptr<Oriented_bounding_box> bbox;
	Eigen::Vector3f normal;
	std::vector<Eigen::Vector3f> sample_points;
	std::vector<size_t> local_path;
	std::vector<size_t> indices_within_bbox;

	bool selected = false;

	Detected_plane_segment(std::shared_ptr<Oriented_bounding_box> bbox, Eigen::Vector3f normal)
		: bbox(bbox), normal(normal)
	{
		this->uuid = generate_uuid();
	}

	bool intersect_ray(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_direction, float& intersection_distance)
	{
		Eigen::Vector3f dir = ray_direction.normalized();
		Eigen::Vector3f p = bbox->get_center() - ray_origin;
		Eigen::Vector3f half_extent = bbox->extent_ / 2.0f;

		float t_min = -std::numeric_limits<float>::infinity();
		float t_max = std::numeric_limits<float>::infinity();

		for (int i = 0; i < 3; i++)
		{
			Eigen::Vector3f axis = bbox->R_.col(i);  // Local OBB axis
			float e = axis.dot(p);               // Projection of p onto axis
			float f = axis.dot(dir);             // Projection of ray direction onto axis

			if (std::abs(f) > 1e-6) {  // Check if ray is not parallel to the OBB plane
				float t1 = (e + half_extent[i]) / f;
				float t2 = (e - half_extent[i]) / f;

				// Ensure t1 is the entry point and t2 is the exit point
				if (t1 > t2) std::swap(t1, t2);

				// Update t_min and t_max for the intersection interval
				t_min = std::max(t_min, t1);
				t_max = std::min(t_max, t2);

				// If the intervals do not overlap, there is no intersection
				if (t_min > t_max) return false;
			}
			else {
				// Ray is parallel to the OBB's plane; check if ray origin is within the extent along this axis
				if (-e - half_extent[i] > 0 || -e + half_extent[i] < 0) {
					return false;  // Ray is outside OBB on this axis
				}
			}
		}

		intersection_distance = t_min;
		return true;
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
