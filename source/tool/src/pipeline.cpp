#include "pipeline.hpp"

#include <CGAL/Point_set_3.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>

//#include "blast/util.hpp"


// Define Kernel and types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> Surface_mesh;

// Kernel and types
typedef Kernel::Point_3 Point;
typedef boost::graph_traits<Surface_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor halfedge_descriptor;

// Skeleton types
typedef CGAL::Mean_curvature_flow_skeletonization<Surface_mesh> Skeletonization;
typedef Skeletonization::Skeleton Skeleton;
typedef Skeleton::vertex_descriptor Skeleton_vertex;
typedef boost::graph_traits<Skeleton>::edge_descriptor Skeleton_edge;

typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef Kernel::Sphere_3 Sphere_3;
typedef CGAL::Point_set_3<Point_3, Vector_3> Point_set;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace pcl
{
	struct PointXYZ;
}

std::vector<Eigen::Vector3f> run_pipeline(const std::string base_pointcloud_file)
{
	spdlog::info("Start running pipeline...");

	pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	load_pcl_pointcloud(base_pointcloud_file, base_cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//downsampled_cloud = base_cloud;
	voxelgrid_downsample(base_cloud, downsampled_cloud);

	//pcl::PointCloud<pcl::Normal>::Ptr out_normals(new pcl::PointCloud<pcl::Normal>);
	//estimate_normals(downsampled_cloud, out_normals);

	// Combine points and normals
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*downsampled_cloud, *out_normals, *cloud_with_normals);

	pcl::PolygonMesh::Ptr pcl_mesh(new pcl::PolygonMesh);
	//poisson_reconstruction(cloud_with_normals, *pcl_mesh);


	std::vector<Eigen::Vector3f> skeleton_vertices;

	if (!skeletonize(*downsampled_cloud, skeleton_vertices))
	{
		return {};
	}

	return skeleton_vertices;
}

bool skeletonize(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, std::vector<Eigen::Vector3f>& out_skeleton_vertices)
{
	spdlog::info("PCL - Saving point cloud to file");
	pcl::io::savePLYFileBinary("mesh.ply", input_cloud);
	Point_set points;

	spdlog::info("CGAL - Loading point cloud from file");
	std::ifstream input("mesh.ply", std::ios::binary); // Load PLY file

	if (!input || !CGAL::IO::read_PLY(input, points)) {
		std::cerr << "Error: cannot read the file mesh.ply" << std::endl;
		return EXIT_FAILURE;
	}

	spdlog::info("Read file");
	spdlog::info("CGAl - Estimating normals");
	CGAL::jet_estimate_normals<CGAL::Sequential_tag>(points, 24);
	
	spdlog::info("CGAl - Average spacing");
	double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, 24);
	// Orientation of normals, returns iterator to first unoriented point

	spdlog::info("CGAl - Orient normals");
	Point_set::iterator unoriented_points_begin = CGAL::mst_orient_normals(points, 24); // Use 24 neighbors

	points.remove(unoriented_points_begin, points.end());
	Surface_mesh mesh;

	spdlog::info("CGAL - Poisson surface reconstruction");
	CGAL::poisson_surface_reconstruction_delaunay
	(points.begin(), points.end(),
		points.point_map(), points.normal_map(),
		mesh, spacing);
	

	// Check if the mesh has any holes (boundary cycles)
	if (CGAL::is_closed(mesh)) {
		spdlog::info("The mesh is closed and bounds a volume.");
	}
	else {
		spdlog::warn("The mesh has holes. Attempting to fill...");

		// Iterate over all boundary cycles and fill them
		for (halfedge_descriptor h : halfedges(mesh)) {
			if (CGAL::is_border(h, mesh)) {
				// Fill each hole using the new function
				PMP::triangulate_and_refine_hole(mesh, h,
					PMP::parameters::vertex_point_map(mesh.points())
					.geom_traits(Kernel()));

				spdlog::info("Hole filled!");
			}
		}

		// Check again if the mesh is now closed
		if (CGAL::is_closed(mesh)) {
			spdlog::info("The mesh is now closed after filling holes.");
		}
		else {
			spdlog::error("The mesh still has open boundaries after hole filling.");
			return false;
		}
	}

	// Create a skeletonization object
	Skeleton skeleton;
	Skeletonization mcs(mesh);

	// Compute skeleton
	spdlog::info("Computing skeleton...");
	mcs.contract_geometry(); // Optional: Contracts the mesh
	mcs.split_faces();       // Optional: Splits long faces
	mcs.contract();          // Main contraction step
	mcs.detect_degeneracies(); // Optional: Detects and removes degenerate edges
	mcs.convert_to_skeleton(skeleton);

	spdlog::info("Skeleton vertices: {}", num_vertices(skeleton));
	for (Skeleton_vertex e : CGAL::make_range(vertices(skeleton))) {
		const Point& p = skeleton[e].point;
		out_skeleton_vertices.push_back(Eigen::Vector3f(p.x(), p.y(), p.z()));
	}

	return true;
}

void poisson_reconstruction(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, pcl::PolygonMesh& out_mesh)
{
	spdlog::info("Poisson reconstruction...");
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth(8); // Set parameters as required
	poisson.setInputCloud(cloud_with_normals);
	pcl::PolygonMesh pcl_mesh;
	poisson.reconstruct(pcl_mesh);
}

bool load_pcl_pointcloud(const std::string base_pointcloud_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	if (base_pointcloud_file.ends_with(".ply") &&
		pcl::io::loadPLYFile<pcl::PointXYZ>(base_pointcloud_file, *cloud) != -1)
	{
		spdlog::info("Successfully loaded ply file...");
		return true;
	}
	else if (base_pointcloud_file.ends_with(".pcd") &&
		pcl::io::loadPCDFile<pcl::PointXYZ>(base_pointcloud_file, *cloud) != -1)
	{
		spdlog::info("Successfully loaded pcd file...");
		return true;
	}

	spdlog::error("Failed to load point cloud file...");
	return false;
}

void voxelgrid_downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
	spdlog::info("Voxel grid downsampling...");

	// Call the PCL function
	pcl::VoxelGrid<pcl::PointXYZ> vox;
	vox.setInputCloud(input_cloud);
	vox.setLeafSize(2.0f, 2.0f, 2.0f);
	vox.filter(*output_cloud);
}

void estimate_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::Normal>::Ptr output_normals)
{
	spdlog::info("Normal estimation...");

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud(input_cloud);
	n.setInputCloud(input_cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*output_normals);
}
