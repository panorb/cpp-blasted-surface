#include <iosfwd>
#include <vector>
#include <blast/point_cloud.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

pcl::PointCloud<pcl::Normal>::Ptr to_pcl_normal_cloud(const std::vector<Eigen::Vector3d>& points)
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud(new pcl::PointCloud<pcl::Normal>);
	cloud->points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		cloud->points[i].normal_x = points[i].x();
		cloud->points[i].normal_y = points[i].y();
		cloud->points[i].normal_z = points[i].z();
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl_point_cloud(const std::vector<Eigen::Vector3d>& points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = points[i].x();
		cloud->points[i].y = points[i].y();
		cloud->points[i].z = points[i].z();
	}

	return cloud;
}

const std::vector<Eigen::Vector3d>& blast::Point_cloud::get_points() const
{
	return points;
}

std::unique_ptr<blast::Point_cloud> from_pcl_point_cloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
	std::vector<Eigen::Vector3d> points;

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

#include <pcl/filters/voxel_grid.h>

void blast::Point_cloud::voxelgrid_downsample(float gridSize)
{
	auto pcl_points = to_pcl_point_cloud(points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	// Call the PCL function
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	vox.setInputCloud(pcl_points);
	vox.setLeafSize(gridSize, gridSize, gridSize);
	vox.filter(*cloud_filtered);

	points.clear();

	for (auto pnt : cloud_filtered->points)
	{
		points.emplace_back(pnt.x, pnt.y, pnt.z);
	}
}

std::vector<std::array<size_t, 3>> blast::Point_cloud::greedy_triangulation(std::vector<Eigen::Vector3d> normals)
{
	auto pcl_points = to_pcl_point_cloud(points);
	auto pcl_normals = to_pcl_normal_cloud(normals);

	auto cloud_with_normals = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
	pcl::concatenateFields(*pcl_points, *pcl_normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);
	
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree);
	gp3.reconstruct(triangles);

	std::vector<std::array<size_t, 3>> triangles_vec;

	for (size_t i = 0; i < triangles.polygons.size(); ++i)
	{
		auto& triangle = triangles.polygons[i];
		std::array<size_t, 3> triangle_indices;
		for (size_t j = 0; j < 3; ++j)
		{
			triangle_indices[j] = triangle.vertices[j];
		}
		triangles_vec.push_back(triangle_indices);
	}

	return triangles_vec;
}

std::vector<Eigen::Vector3d> blast::Point_cloud::estimate_normals(float radius)
{
	auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
	auto pcl_points = to_pcl_point_cloud(points);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
	tree->setInputCloud(pcl_points);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(pcl_points);
	n.setSearchMethod(tree);
	n.setKSearch(8);
	n.compute(*normals);

	std::vector<Eigen::Vector3d> normals_vec;
	for (const auto& normal : normals->points)
	{
		normals_vec.emplace_back(normal.normal_x, normal.normal_y, normal.normal_z);
	}

	return normals_vec;
}

Eigen::Vector3d blast::Point_cloud::compute_min_bound() const
{
	if (points.empty()) {
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}
	return std::accumulate(
		points.begin(), points.end(), points[0],
		[](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
			return a.array().min(b.array()).matrix();
		});
}

Eigen::Vector3d blast::Point_cloud::compute_max_bound() const
{
	if (points.empty()) {
		return Eigen::Vector3d(0.0, 0.0, 0.0);
	}
	return std::accumulate(
		points.begin(), points.end(), points[0],
		[](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
			return a.array().max(b.array()).matrix();
		});

}


