#include "debug.hpp"

#include <algorithm>

namespace blast
{
	std::vector<size_t> get_k_highest_points(Point_cloud& cloud, int k, float ignore_distance)
	{
		// Find the indices of the k highest points (z coordinate)
		auto points = cloud.get_points_f();
		std::vector<std::pair<size_t, float>> indices_and_z;

		for (size_t i = 0; i < points.size(); ++i) {
			indices_and_z.emplace_back(i, points[i].z());
		}

		std::sort(indices_and_z.begin(), indices_and_z.end(),
			[](const auto& a, const auto& b) {
				return a.second > b.second;
			});

		std::vector<size_t> result;
		for (int i = 0; i < indices_and_z.size(); ++i) {
			bool should_ignore = false;
			
			for (size_t prev_result_idx : result)
			{
				if ((points[i] - points[prev_result_idx]).norm() < ignore_distance)
				{
					should_ignore = true;
					break;
				}
			}

			if (should_ignore) continue;

			result.push_back(indices_and_z[i].first);
			if (result.size() >= k)
				break;
		}

		return result;
	}

	std::vector<size_t> get_k_lowest_points(Point_cloud& cloud, int k, float ignore_distance)
	{
		// Find the indices of the k lowest points (z coordinate)
		auto points = cloud.get_points_f();
		std::vector<std::pair<size_t, float>> indices_and_z;
		for (size_t i = 0; i < points.size(); ++i) {
			indices_and_z.emplace_back(i, points[i].z());
		}
		std::sort(indices_and_z.begin(), indices_and_z.end(),
			[](const auto& a, const auto& b) {
				return a.second < b.second;
			});
		std::vector<size_t> result;

		for (int i = 0; i < indices_and_z.size(); ++i) {
			bool should_ignore = false;

			float closest_distance = std::numeric_limits<float>::max();

			for (size_t prev_result_idx : result)
			{
				float point_distance = (points[i] - points[prev_result_idx]).norm();

				closest_distance = std::min(point_distance, closest_distance);

				if (point_distance < ignore_distance)
				{
					should_ignore = true;
				}
			}

			if (should_ignore) continue;

			result.push_back(indices_and_z[i].first);
			if (result.size() >= k)
				break;
		}
		return result;
	}
}
