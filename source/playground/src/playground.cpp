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

#include "meshview/meshview.hpp"

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
 //   std::cout << "Loading input point cloud..." << std::endl;

 //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //   if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1) // Load point cloud
 //   {
 //       PCL_ERROR("Couldn't read file input.pcd \n");
 //       return (-1);
 //   }

	//std::cout << "Loaded " << cloud->width * cloud->height << " data points from input.pcd" << std::endl;
	//std::cout << "Voxel grid downsampling..." << std::endl;
 //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

 //   // Call the PCL function
 //   pcl::VoxelGrid<pcl::PointXYZ> vox;
 //   vox.setInputCloud(cloud);
 //   vox.setLeafSize(3.0f, 3.0f, 3.0f);
 //   vox.filter(*cloud_filtered);

 //   std::cout << "Estimating normals..." << std::endl;

 //   // Normal estimation (optional but recommended)
 //   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
 //   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
 //   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
 //   tree->setInputCloud(cloud_filtered);
 //   n.setInputCloud(cloud_filtered);
 //   n.setSearchMethod(tree);
 //   n.setKSearch(20);
 //   n.compute(*normals);

 //   // Combine points and normals
 //   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
 //   pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);

 //   // Create search tree
 //   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
 //   tree2->setInputCloud(cloud_with_normals);

	//// Poisson reconstruction
 //   //std::cout << "Poisson reconstruction..." << std::endl;
 //   //
 //   //pcl::Poisson<pcl::PointNormal> poisson;
 //   //poisson.setDepth(8); // Set parameters as required
 //   //poisson.setInputCloud(cloud_with_normals);
 //   //pcl::PolygonMesh pcl_mesh;
 //   //poisson.reconstruct(pcl_mesh);

	//std::cout << "Greedy projection triangulation..." << std::endl;
 //   pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh pcl_mesh;
 //   // Set the maximum distance between connected points (maximum edge length)
 //   gp3.setSearchRadius(7.0f);

 //   // Set typical values for the parameters
 //   gp3.setMu(2.5);
 //   gp3.setMaximumNearestNeighbors(100);
 //   gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
 //   gp3.setMinimumAngle(M_PI / 18); // 10 degrees
 //   gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
 //   gp3.setNormalConsistency(false);

	//gp3.setInputCloud(cloud_with_normals);
	//gp3.setSearchMethod(tree2);

 //   gp3.reconstruct(pcl_mesh);

 //   // Save the mesh in PLY format
 //   pcl::io::savePLYFile("mesh.ply", pcl_mesh);

 //   std::cout << "=== PCL END" << std::endl;
    std::cout << "=== CGAL BEGIN" << std::endl;

    std::cout << "Reading .ply..." << std::endl;

    std::ifstream input("mesh.ply"); // Load PLY file
    Surface_mesh mesh;

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
    // Create a meshview::Viewer object
    meshview::Viewer viewer;

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

    std::cout << "Skeleton and mapping written to files." << std::endl;
    return EXIT_SUCCESS;
}
