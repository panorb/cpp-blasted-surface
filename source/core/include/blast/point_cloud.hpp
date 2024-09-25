#pragma once

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Core>

namespace blast
{
	class Point_cloud
	{
	public:
		Point_cloud() = default;
		explicit Point_cloud(const std::vector<Eigen::Vector3d>& points) : points(points) {};
		const std::vector<Eigen::Vector3d>& get_points() const;
		const std::vector<Eigen::Vector3f> get_points_f() const;

		static std::unique_ptr<Point_cloud> load_pcd_file(const std::string& file_path);
		static std::unique_ptr<Point_cloud> load_ply_file(const std::string& file_path);
		void voxelgrid_downsample(float gridSize);
		std::vector<std::array<size_t, 3>> greedy_triangulation(std::vector<Eigen::Vector3d>);
		std::vector<Eigen::Vector3d> estimate_normals(float radius);
		Eigen::Vector3d compute_min_bound() const;
		Eigen::Vector3d compute_max_bound() const;

	private:
		std::vector<Eigen::Vector3d> points;
	};
}
