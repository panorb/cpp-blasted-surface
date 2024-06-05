#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace blast
{
	class Point_cloud
	{
	public:
		Point_cloud() = default;
		explicit Point_cloud(const std::vector<glm::vec3>& points) : points(points) {};
		[[nodiscard]] std::vector<glm::vec3> get_points() const;

		static std::unique_ptr<blast::Point_cloud> from_pcl_point_cloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);
		static std::unique_ptr<Point_cloud> load_pcd_file(const std::string& file_path);
		static std::unique_ptr<Point_cloud> load_ply_file(const std::string& file_path);
	private:
		std::vector<glm::vec3> points;
	};
}