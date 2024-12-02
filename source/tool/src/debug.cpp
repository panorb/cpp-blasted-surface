#include "debug.hpp"

namespace blast
{
	std::vector<size_t> get_k_highest_points(Point_cloud& cloud, int k)
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
		for (int i = 0; i < k && i < indices_and_z.size(); ++i) {
			result.push_back(indices_and_z[i].first);
		}

		return result;
	}

	std::vector<size_t> get_k_lowest_points(Point_cloud& cloud, int k)
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
		for (int i = 0; i < k && i < indices_and_z.size(); ++i) {
			result.push_back(indices_and_z[i].first);
		}
		return result;
	}
}
