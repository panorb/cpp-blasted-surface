#include <blast/point_cloud.hpp>
#include <catch.hpp>

TEST_CASE("Point clouds can be created from point list", "[point_cloud]") {
	std::vector<Eigen::Vector3d> points = {{-1.f, 0.f, 1.f}, {1.f, -1.f, 0.f}, {0.f, 1.f, -1.f}};
	const blast::Point_cloud point_cloud{points};
	REQUIRE(point_cloud.get_points().size() == points.size());
	REQUIRE(point_cloud.get_points()[0] == points[0]);
}

TEST_CASE("Point clouds can be loaded from ply file", "[point_cloud, pcl, io]") {
	const std::unique_ptr<blast::Point_cloud> point_cloud = blast::Point_cloud::load_ply_file("files/cube.ply");
	REQUIRE(point_cloud->get_points().size() == 30246);
}
