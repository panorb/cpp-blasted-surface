#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <memory>

namespace blast
{
	class Point_cloud
	{
	public:
		Point_cloud() = default;
		explicit Point_cloud(const std::vector<glm::vec3>& points) : points(points) {};
		const std::vector<glm::vec3>& get_points() const;

		static std::unique_ptr<Point_cloud> load_pcd_file(const std::string& file_path);
		static std::unique_ptr<Point_cloud> load_ply_file(const std::string& file_path);
		void voxelgrid_downsample(float gridSize);
		std::vector<std::array<size_t, 3>> greedy_triangulation(std::vector<glm::vec3>);
		std::vector<glm::vec3> estimate_normals(float radius);

	private:
		std::vector<glm::vec3> points;
	};
}