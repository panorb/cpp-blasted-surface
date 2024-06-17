#include <fstream>
#include <blast/point_cloud.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


std::vector<glm::vec3> blast::Point_cloud::get_points() const
{
	return points;
}

std::unique_ptr<blast::Point_cloud> from_pcl_point_cloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
	std::vector<glm::vec3> points;

	points.reserve(pcl_cloud.points.size());

	for (const auto pnt : pcl_cloud.points)
	{
		points.emplace_back(pnt.x, pnt.y, pnt.z);
	}

	return std::make_unique<blast::Point_cloud>(points);
}

std::unique_ptr<blast::Point_cloud> blast::Point_cloud::load_pcd_file(const std::string& file_path)
{
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(file_path, *cloud);

	return from_pcl_point_cloud(*cloud);
}

std::unique_ptr<blast::Point_cloud> blast::Point_cloud::load_ply_file(const std::string& file_path)
{
	
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(file_path, *cloud);

	return from_pcl_point_cloud(*cloud);
}


