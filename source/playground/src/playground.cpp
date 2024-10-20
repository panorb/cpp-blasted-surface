#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/PolygonMesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/IO/PLY.h>

#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/measure.h> // For is_closed

#include <fstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <spdlog/spdlog.h>

#include "blast/detected_plane_segment.hpp"
#include "blast/graph.hpp"
#include "blast/planes/oliveira_planes.hpp"
#include "blast/sampler/grid_sampler.hpp"
#include "meshview/meshview.hpp"
#include <blast/greedy_optimizer.hpp>

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

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char** argv)
{
    std::cout << "Loading input point cloud..." << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("input2.pcd", *cloud) == -1) // Load point cloud
    {
        PCL_ERROR("Couldn't read file input.pcd \n");
        return (-1);
    }

	std::cout << "Loaded " << cloud->width * cloud->height << " data points from input.pcd" << std::endl;
	std::cout << "Voxel grid downsampling..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Call the PCL function
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(2.0f, 2.0f, 2.0f);
    vox.filter(*cloud_filtered);

    std::cout << "Estimating normals..." << std::endl;

    // Normal estimation (optional but recommended)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    // Combine points and normals
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

	//// Poisson reconstruction
    std::cout << "Poisson reconstruction..." << std::endl;

	pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(12); // Set parameters as required
    poisson.setInputCloud(cloud_with_normals);
    pcl::PolygonMesh pcl_mesh;
    poisson.reconstruct(pcl_mesh);

	//std::cout << "Greedy projection triangulation..." << std::endl;
 //   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh pcl_mesh;
 //   // Set the maximum distance between connected points (maximum edge length)
 //   gp3.setSearchRadius(15.0f);

 //   // Set typical values for the parameters
 //   gp3.setMu(4.f);
 //   //gp3.setMaximumNearestNeighbors(100);
 //   gp3.setMaximumSurfaceAngle(M_PI / 2); // 45 degrees
 //   gp3.setMinimumAngle(M_PI / 24); // 10 degrees
 //   gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
 //   gp3.setNormalConsistency(false);

	//gp3.setInputCloud(cloud_with_normals);
	//gp3.setSearchMethod(tree2);

 //   gp3.reconstruct(pcl_mesh);

	// Output number of vertices and faces
	std::cout << "Mesh has " << pcl_mesh.cloud.width * pcl_mesh.cloud.height << " vertices and " << pcl_mesh.polygons.size() << " faces." << std::endl;

    // Save the mesh in PLY format
    pcl::io::savePLYFileBinary("mesh.ply", pcl_mesh);

	std::cout << "Mesh saved to mesh.ply" << std::endl;

    std::cout << "=== PCL END" << std::endl;
    std::cout << "=== CGAL BEGIN" << std::endl;

    std::cout << "Reading .ply..." << std::endl;

    std::ifstream input("mesh.ply", std::ios::binary); // Load PLY file
    Surface_mesh mesh;

    if (!input)
    {
		std::cerr << "Error: cannot read the file mesh.ply filestream" << std::endl;
		return EXIT_FAILURE;
    }

    if (!input || !CGAL::IO::read_PLY(input, mesh)) {
        std::cerr << "Error: cannot read the file mesh.ply" << std::endl;
        return EXIT_FAILURE;
    }

    // Print some information about the mesh
    std::cout << "Number of vertices: " << CGAL::num_vertices(mesh) << std::endl;
    std::cout << "Number of faces: " << CGAL::num_faces(mesh) << std::endl;

    PMP::remove_isolated_vertices(mesh);

    // Check if the mesh has any holes (boundary cycles)
    if (CGAL::is_closed(mesh)) {
        std::cout << "The mesh is closed and bounds a volume." << std::endl;
    }
    else {
        std::cout << "The mesh has holes. Attempting to fill..." << std::endl;

        // Iterate over all boundary cycles and fill them
        for (halfedge_descriptor h : halfedges(mesh)) {
            if (CGAL::is_border(h, mesh)) {
                // Fill each hole using the new function
                PMP::triangulate_and_refine_hole(mesh, h,
                    PMP::parameters::vertex_point_map(mesh.points())
                    .geom_traits(Kernel()));

                std::cout << "Hole filled!" << std::endl;
            }
        }

        // Check again if the mesh is now closed
        if (CGAL::is_closed(mesh)) {
            std::cout << "The mesh is now closed after filling holes." << std::endl;
        }
        else {
            std::cerr << "The mesh still has open boundaries after hole filling." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Create a skeletonization object
    Skeleton skeleton;
    Skeletonization mcs(mesh);

    // Compute skeleton
    std::cout << "Computing skeleton..." << std::endl;
    mcs.contract_geometry(); // Optional: Contracts the mesh
    mcs.split_faces();       // Optional: Splits long faces
    mcs.contract();          // Main contraction step
    mcs.detect_degeneracies(); // Optional: Detects and removes degenerate edges
    mcs.convert_to_skeleton(skeleton);

    std::cout << "Skeleton vertices: " << num_vertices(skeleton) << std::endl;


    std::cout << "Extracting planes..." << std::endl;

    std::vector<Eigen::Vector3d> points_vec;
    for (const auto& point : cloud_filtered->points)
    {
        points_vec.emplace_back(point.x, point.y, point.z);
    }

    std::vector<Eigen::Vector3d> normals_vec;
    for (const auto& normal : normals->points)
    {
        normals_vec.emplace_back(normal.normal_x, normal.normal_y, normal.normal_z);
    }

    Oliveira_plane_segmenter oliveira_plane_segmenter;
    oliveira_plane_segmenter.from_arrays(points_vec, normals_vec);
    auto bbox_planes = oliveira_plane_segmenter.execute();

    std::cout << "Found " << bbox_planes.size() << " planes." << std::endl;


    // Create a meshview::Viewer object
    meshview::Viewer viewer;

    std::vector<Detected_plane_segment> detected_planes;

    for (const auto& plane : bbox_planes)
    {
        std::cout << "Plane: " << plane << std::endl;

		// Check if the plane is colliding with other planes
		bool colliding = false;

		/*for (const auto& other_plane : bbox_planes)
		{
            if (plane == other_plane)
            {
                continue;
            }

            // The logic for checking collsion
            plane->scale(0.001f, plane->get_center());
			other_plane->scale(0.001f, other_plane->get_center());

			if (plane->collides_with(*other_plane))
			{
				colliding = true;
				break;
			}
		}*/

		// If the plane is colliding, skip it
        /*if (colliding)
		{
			continue;
		}*/

		// Save distance of nearest point in skeleton to plane
        Point& nearest_point = skeleton[0].point;
		double min_distance = std::numeric_limits<double>::max();

        // Find nearest point in skeleton to plane
        for (Skeleton_vertex e : CGAL::make_range(vertices(skeleton))) {
			const Point& p = skeleton[e].point;
			// std::cout << "Point: " << p << std::endl;

            Eigen::Vector3f center = plane->get_center();

			double distance = std::sqrt(
				std::pow(p.x() - center.x(), 2) +
				std::pow(p.y() - center.y(), 2) +
				std::pow(p.z() - center.z(), 2)
			);

			if (distance < min_distance)
			{
				min_distance = distance;
				nearest_point = p;
			}
		}

		// Calculate which normal to use
		Eigen::Vector3f normal = plane->R_ * Eigen::Vector3f(0, 0, 1);

    	// Calculate wether normal needs to be flipped to point away from the skeleton

		Eigen::Vector3f skeleton_to_plane = Eigen::Vector3f(
			plane->get_center().x() - nearest_point.x(),
			plane->get_center().y() - nearest_point.y(),
			plane->get_center().z() - nearest_point.z()
		);

		if (skeleton_to_plane.dot(normal) < 0)
		{
			normal = -normal;
		}

		detected_planes.push_back(Detected_plane_segment{
			plane,
			normal,
            {}
		});

		detected_planes.back().sample_points = sample_points_on_grid(detected_planes.back(), 0.1f, 3.0f);


        // Draw the plane bbox in viewer
        auto box_points = plane->get_box_points();
        meshview::Points vertices{ box_points.size(), 3 };

        for (size_t i = 0; i < box_points.size(); ++i)
        {
            vertices.row(i)(0) = box_points[i].x();
            vertices.row(i)(1) = box_points[i].y();
            vertices.row(i)(2) = box_points[i].z();
        }

        meshview::Triangles triangles{ 12, 3 };
        triangles << 0, 1, 2,
            0, 2, 3,
            0, 4, 5,
            0, 5, 1,
            1, 5, 6,
            1, 6, 2,
            2, 6, 7,
            2, 7, 3,
            3, 7, 4,
            3, 4, 0,
            4, 7, 6,
            4, 6, 5;


        float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
        plane->color_ = Eigen::Vector3f(r, g, b);

        viewer.add_mesh("oliveira", vertices, triangles, r, g, b);

		// Draw the normal in viewer
		Eigen::Vector3f start = plane->get_center();
		Eigen::Vector3f end = start + normal * 10;

		viewer.add_line(start, end, Eigen::Vector3f(1, 0, 0));
    }

	for (int i = 0; i < detected_planes.size(); i++)
	{
        blast::Graph graph;
        auto& plane = detected_planes[i];

        // Add sample points to graph
        for (int j = 0; j < plane.sample_points.size(); ++j)
        {
            auto& point = plane.sample_points[j];
            graph.add_node(std::make_shared<blast::Node>(std::format("P{}S{}", i, j)));
        }

        // Add edges to graph
        for (int j = 0; j < plane.sample_points.size(); ++j)
        {
            for (int k = 0; k < plane.sample_points.size(); ++k)
            {
                if (j != k && !graph.exists_edge(j, k))
                {
                    auto& point1 = plane.sample_points[j];
                    auto& point2 = plane.sample_points[k];
                    auto distance = (point1 - point2).norm();
                    graph.add_undirected_edge(j, k, distance);
                }
            }
        }

        blast::Greedy_optimizer greedy_optimizer{ &graph };
        std::vector<size_t> path = greedy_optimizer.execute();

        // Prepare outputting resulting path index sequence e.g. 0 -> 1 -> 2 -> 3
        std::string r = "";

        for (auto& node : path)
        {
            r += std::format("{0:d} -> ", node);
        }

        // Remove last arrow from output string
        r = r.substr(0, r.size() - 4);
        spdlog::info("Path: {0}", r);

        // Draw path in viewer
        for (int j = 0; j < path.size() - 1; ++j)
        {
            auto& point1 = plane.sample_points[path[j]];
            auto& point2 = plane.sample_points[path[j + 1]];

            viewer.add_line(point1.cast<float>(), point2.cast<float>(), Eigen::Vector3f(1.0, 0.0, 0.0));
        }
	}

    // Draw surface mesh in viewer
    // Extract vertices and faces from the Surface_mesh
    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::vector<int>> faces;

    for (auto v : mesh.vertices()) {
        const auto& p = mesh.point(v);
        vertices.emplace_back(p.x(), p.y(), p.z());
    }

    for (auto f : mesh.faces()) {
        std::vector<int> face;
        for (auto v : CGAL::vertices_around_face(mesh.halfedge(f), mesh)) {
            face.push_back(v.idx());
        }
        faces.push_back(face);
    }


    meshview::Points viewer_pts{ vertices.size(), 3 };
	meshview::Triangles viewer_tri{ faces.size(), 3 };

	for (size_t i = 0; i < vertices.size(); i++) {
        viewer_pts.row(i)(0) = vertices[i](0);
        viewer_pts.row(i)(1) = vertices[i](1);
        viewer_pts.row(i)(2) = vertices[i](2);
	}

    for (size_t i = 0; i < faces.size(); i++) {
        viewer_tri.row(i)(0) = faces[i][0];
        viewer_tri.row(i)(1) = faces[i][1];
        viewer_tri.row(i)(2) = faces[i][2];
    }

    // Add the mesh to the viewer
    viewer.add_mesh("example",viewer_pts, viewer_tri,
        0.f, 1.0f, 1.0f);

    // Add lines for every edge in skeleton
    for (Skeleton_edge e : CGAL::make_range(edges(skeleton))) {
        const Point& s = skeleton[source(e, skeleton)].point;
        const Point& t = skeleton[target(e, skeleton)].point;

        viewer.add_line(
            Eigen::Vector3f(s.x(), s.y(), s.z()),
            Eigen::Vector3f(t.x(), t.y(), t.z())
        );
    }

    // Show the viewer
    viewer.show();
    return EXIT_SUCCESS;
}
