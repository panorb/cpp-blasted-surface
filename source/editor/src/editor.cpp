#include <meshview/meshview.hpp>
#include <meshview/meshview_imgui.hpp>
#include <Eigen/Core>
#include "Eigen/Geometry"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>

// Eigene Includes
#include <blast/point_cloud.hpp>
//#include <blast/oliveira_planes.hpp>
#include <blast/voxel_grid.hpp>
#include <spdlog/spdlog.h>
#include <filesystem>
// #include <spdlog/spdlog.h>

#include <blast/planes/oliveira_planes.hpp>

#include "blast/ant_colony_optimizer.hpp"
#include "blast/detected_plane_segment.hpp"
#include "blast/graph.hpp"
#include "blast/greedy_optimizer.hpp"
#include "blast/sampler/grid_sampler.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>

using namespace meshview;

std::unique_ptr<blast::Point_cloud> base_point_cloud = nullptr;
size_t base_point_cloud_index = 0;

Oliveira_plane_segmenter oliveira_planes;

std::shared_ptr<blast::Voxel_grid> displayed_voxel_grid;

std::shared_ptr<blast::Voxel_grid> model_voxel_grid;
std::shared_ptr<blast::Voxel_grid> safety_distance_voxel_grid;
std::shared_ptr<blast::Voxel_grid> max_viewing_range_voxel_grid;
std::shared_ptr<blast::Voxel_grid> via_point_voxel_grid;

PointCloud* voxel_grid_points;
size_t voxel_grid_points_index = 0;

int candidate_amount = 10;
float potential_field_max_distance = 12.0f;

float voxel_size = 10.0f;

bool voxel_meshes_visible = true;
bool voxel_points_visible = false;


bool is_overlapping(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> inbetween_points, Eigen::Vector3f p2)
{
    return false;
}

void update_displayed_voxel_grid(Viewer& viewer, Vector3f color = Vector3f(1.0, 0.8, 0.8))
{
    if (voxel_grid_points != nullptr)
    {
        viewer.point_clouds.erase(viewer.point_clouds.begin() + voxel_grid_points_index);
        voxel_grid_points = nullptr;
    }

	viewer.delete_all("voxelgrid");

    auto voxels = displayed_voxel_grid->get_voxels();
    auto voxel_count = voxels.size();

    Points points{ voxel_count, 3 };

    for (size_t i = 0; i < voxel_count; ++i)
    {
        auto voxel = voxels[i];
        spdlog::info("Voxel [{0:d}, {1:d}, {2:d}]", voxel.grid_index.x(), voxel.grid_index.y(), voxel.grid_index.z());

        Eigen::Vector3d voxel_center = displayed_voxel_grid->get_voxel_center_coordinate(voxel.grid_index);

        points.row(i)(0) = voxel_center.x();
        points.row(i)(1) = voxel_center.y();
        points.row(i)(2) = voxel_center.z();

        &viewer.add_cube("voxelgrid", voxel_center.cast<float>(), voxel_size, color).enable(voxel_meshes_visible);
    }

    voxel_grid_points_index = viewer.point_clouds.size();
    voxel_grid_points = &viewer.add_point_cloud(points, color.x(), color.y(), color.z()).enable(voxel_points_visible);
}

int main(int argc, char** argv)
{
	Viewer viewer;
	viewer.draw_axes = true;
	viewer.camera.dist_to_center = 5.f;
    viewer.cull_face = false;

	std::vector<std::string> example_files;
    std::string selected_example_file = "knochen-komplett.pcd";
    // std::vector<std::shared_ptr<Oriented_bounding_box>> found_planes;
	std::vector<Detected_plane_segment> detected_planes;

    viewer.on_gui = [&]() -> bool {
        bool redraw_meshes = false;

        ImGui::ShowDemoWindow();

    	ImGui::SetNextWindowSize(ImVec2(400, 800), ImGuiCond_Once);

        ImGui::Begin("Editor");

        if (ImGui::Button("Clear all"))
        {
	        viewer.meshes.clear();
            viewer.point_clouds.clear();
            base_point_cloud = nullptr;
        }

        // ImGui::Combo

        if (ImGui::BeginCombo("Pointcloud file", selected_example_file.c_str()))
        {
            example_files.clear();
            // Get all files in directory "/examples"
            for (const auto& entry : std::filesystem::directory_iterator(R"(.\examples)"))
            {
                auto res = std::filesystem::path(entry.path()).filename().string();
				example_files.push_back(res);

                if (ImGui::Selectable(example_files.back().c_str(), example_files.back() == selected_example_file))
                {
                    selected_example_file = example_files.back();
                }
            }

            ImGui::EndCombo();
        }

        // ImGui::InputText("Pointcloud file", &filename);
        if (ImGui::Button("Load point cloud"))
        {
			if (!selected_example_file.empty())
			{
                if (base_point_cloud != nullptr)
                {
					viewer.point_clouds.erase(viewer.point_clouds.begin() + base_point_cloud_index);
                }

				base_point_cloud_index = viewer.point_clouds.size();

                if (selected_example_file.ends_with(".ply"))
                {
                    base_point_cloud = blast::Point_cloud::load_ply_file("./examples/" + selected_example_file);
				}
				else if (selected_example_file.ends_with(".pcd"))
				{
					base_point_cloud = blast::Point_cloud::load_pcd_file("./examples/" + selected_example_file);
				}
				else
				{
					//spdlog::error("Unsupported file format");
                    return false;
				}

                size_t point_count = base_point_cloud->get_points().size();
                Points points{point_count, 3};

                for (size_t i = 0; i < point_count; ++i)
				{
					points.row(i)(0) = base_point_cloud->get_points()[i][0];
                    points.row(i)(1) = base_point_cloud->get_points()[i][1];
                    points.row(i)(2) = base_point_cloud->get_points()[i][2];
				}

                viewer.add_point_cloud(points, 0.f, 1.0f, 1.0f);
                redraw_meshes = true;
			}
	        
        }

        if (ImGui::Button("Downsample using voxel grid")) {
            base_point_cloud->voxelgrid_downsample(3.0f);

            viewer.point_clouds.clear();

            size_t point_count = base_point_cloud->get_points().size();
            Points points{ point_count, 3 };

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = base_point_cloud->get_points()[i][0];
                points.row(i)(1) = base_point_cloud->get_points()[i][1];
                points.row(i)(2) = base_point_cloud->get_points()[i][2];
            }

            viewer.add_point_cloud(points, 0.f, 1.0f, 1.0f);

            redraw_meshes = true;
        }


        if (ImGui::Button("Greedy Triangulation"))
        {
            
			viewer.meshes.clear();

			std::vector<Eigen::Vector3d> points = base_point_cloud->get_points();
            std::vector<Eigen::Vector3d> p_normals = base_point_cloud->estimate_normals(25.f);
            auto triangles_vec = base_point_cloud->greedy_triangulation(p_normals);

            Points vertices{points.size(), 3};
            for (size_t i = 0; i < points.size(); ++i)
			{
                vertices.row(i)(0) = points[i].x();
                vertices.row(i)(1) = points[i].y();
            	vertices.row(i)(2) = points[i].z();
			}

            Triangles triangles{triangles_vec.size(), 3};
            Points normals{ triangles_vec.size(), 3 };

        	for (size_t i = 0; i < triangles_vec.size(); ++i)
            {
                triangles.row(i)(0) = triangles_vec[i][0];
                triangles.row(i)(1) = triangles_vec[i][1];
            	triangles.row(i)(2) = triangles_vec[i][2];

                // Normal 1
                auto normal1 = p_normals[triangles_vec[i][0]];
                auto normal2 = p_normals[triangles_vec[i][1]];
                auto normal3 = p_normals[triangles_vec[i][2]];

                normals.row(i)(0) = normal1.cast<float>().x();
                normals.row(i)(1) = normal1.cast<float>().y();
                normals.row(i)(2) = normal1.cast<float>().z();
            }

            //Points vw_normals = Eigen::Map<Points>(reinterpret_cast<float*>(normals.data()), normals.size(), 3);
            // vw_normals *= -1.0f;

            auto& pyra2 = viewer
                .add_mesh(
                    "triangulation",
                    vertices,
                    triangles,
                    0.f, 1.0f, 1.0f,
                    vertices)
                .set_shininess(32.f);
                //.translate(Vector3f(3.f, 3.f, 0.f));


			// blast::Point_cloud::greedy_triangulation(points, normals, vertices, triangles);

            
			redraw_meshes = true;
	        
        }

        ImGui::Separator();

		ImGui::InputFloat("Voxel size", &voxel_size);

        if (ImGui::Button("Create voxel grid based on point cloud"))
        {
            model_voxel_grid = blast::Voxel_grid::create_from_point_cloud(*base_point_cloud, voxel_size);
			displayed_voxel_grid = model_voxel_grid;

            //auto slightly_expanded_grid = blast::dilate_grid(*displayed_voxel_grid, 2);
            //auto strongly_expanded_grid = blast::dilate_grid(*displayed_voxel_grid, 3);
            // displayed_voxel_grid = blast::subtract_grids(*strongly_expanded_grid, *slightly_expanded_grid);

            update_displayed_voxel_grid(viewer, Vector3f(1, 0.8, 0.8));
            redraw_meshes = true;
        }

        if (ImGui::Button("Create voxel grid with safety distance"))
        {
			safety_distance_voxel_grid = blast::dilate_grid(*model_voxel_grid, 1);
			displayed_voxel_grid = safety_distance_voxel_grid;

			update_displayed_voxel_grid(viewer, Vector3f(0.77, 0.87, 0.7));
            redraw_meshes = true;
        }

        if (ImGui::Button("Create voxel grid with max viewing range"))
        {
			max_viewing_range_voxel_grid = blast::dilate_grid(*model_voxel_grid, 3);
			displayed_voxel_grid = max_viewing_range_voxel_grid;

            update_displayed_voxel_grid(viewer);
			redraw_meshes = true;
        }

        if (ImGui::Button("Subtract max viewing range from safety distance grid"))
        {
			via_point_voxel_grid = blast::subtract_grids(*max_viewing_range_voxel_grid, *safety_distance_voxel_grid);
			displayed_voxel_grid = via_point_voxel_grid;

			update_displayed_voxel_grid(viewer);
			redraw_meshes = true;
        }

        if (ImGui::Button("Toggle voxel mesh visibility"))
        {
			/*for (auto& mesh : voxel_grid_meshes)
			{
				voxel_meshes_visible = !voxel_meshes_visible;
                mesh->enable(!mesh->enabled);
			}

			if (voxel_grid_meshes[0]->enabled) {
				voxel_grid_points->enable(false);
			}*/
            voxel_points_visible = false;
        }

        if (ImGui::Button("Toggle voxel center point visibility"))
        {
			voxel_points_visible = !voxel_points_visible;
            voxel_grid_points->enable(!voxel_grid_points->enabled);

            if (voxel_grid_points->enabled) {
                /*for (auto& mesh : voxel_grid_meshes)
                {
                    mesh->enable(false);
                }*/
				voxel_meshes_visible = false;
            }
        }

        ImGui::InputInt("Candidate amount", &candidate_amount);
		ImGui::InputFloat("Potential field max distance", &potential_field_max_distance);

        if (ImGui::Button("Generate via-point candidates from voxel grid"))
        {
			auto via_points = blast::generate_via_point_candidates(*displayed_voxel_grid, candidate_amount, potential_field_max_distance);
			spdlog::info("Generated {0:d} via points", via_points.size());

            viewer.delete_all("candidates");

			for (auto& via_point : via_points)
			{
				auto point = via_point.point;
				auto direction = via_point.direction;

                viewer.add_sphere(
	                "candidates",
	                point.cast<float>(),
	                2.0f, Vector3f(1.0f, 0.0f, 0.0f)
                );


				viewer.add_line(
					point.cast<float>(),
					(point + direction * 8.f).cast<float>(),
					Vector3f(1.0f, 0.0f, 0.0f)
				);

			}

            redraw_meshes = true;
        }

        ImGui::Separator();
		ImGui::Text("Oliveira Plane Segmentation");

        oliveira_planes.render_controls();
        if (ImGui::Button("Oliveira: Extract Planes"))
        {
            std::vector<Eigen::Vector3d> points = base_point_cloud->get_points();
            std::vector<Eigen::Vector3d> normals = base_point_cloud->estimate_normals(25.f);

            oliveira_planes.from_arrays(points, normals);

            auto planes = oliveira_planes.execute();
            // Visualize planes
            for (int i = 0; i < planes.size(); ++i)
			{
                auto& plane = planes[i];
                Eigen::Vector3f normal = plane->R_* Eigen::Vector3f(0, 0, plane->extent_(2));
                normal.normalize();

                // Add to detected planes vector
				detected_planes.push_back(Detected_plane_segment(plane, normal, {}));

				auto box_points = plane->get_box_points();
				Points vertices{box_points.size(), 3};

				for (size_t i = 0; i < box_points.size(); ++i)
				{
                    vertices.row(i)(0) = box_points[i].x();
                    vertices.row(i)(1) = box_points[i].y();
                    vertices.row(i)(2) = box_points[i].z();
				}
                 
				Triangles triangles{12, 3};
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
            	// viewer.add_line(plane->get_center(), plane->get_center() + (2 * normal), Eigen::Vector3f(0.0, 1.0, 0.0));
                
			}
        }

        if (ImGui::Button("Planes: Sample points on grid"))
        {
			for (int i = 0; i < detected_planes.size(); ++i)
			{
                auto& plane = detected_planes[i];
                auto pt_indices = plane.bbox->get_point_indices_within_bounding_box(base_point_cloud->get_points_f());

				// log all indices
				std::string indices = "";
				for (auto& idx : pt_indices)
				{
					indices += std::format("{0:d}, ", idx);
				}

				// Remove last comma
				indices = indices.substr(0, indices.size() - 2);

                // Look which side of the plane has more of the points
				Eigen::Vector3f normal = plane.bbox->R_ * Eigen::Vector3f(0, 0, plane.bbox->extent_(2));

				// Count points on each side of the plane
				int left = 0;
				int right = 0;

				for (auto& idx : pt_indices)
				{
					auto& point = base_point_cloud->get_points()[idx];
					auto d = point - plane.bbox->center_.cast<double>();
					auto dot = d.dot(normal.cast<double>());

					if (dot > 0)
					{
						right++;
					}
					else
					{
						left++;
					}
				}

				if (left > right)
				{
					plane.normal = -normal.normalized();
				}
				else
				{
					plane.normal = normal.normalized();
				}

                viewer.add_line(plane.bbox->get_center(), plane.bbox->get_center() + (5 * plane.normal), Eigen::Vector3f(0.0, 1.0, 0.0));

                spdlog::info("----------------------------");
				spdlog::info("Right: {0}", right);
				spdlog::info("Left: {0}", left);

                plane.sample_points = sample_points_on_grid(plane, 10.0, 30.0);
                // plane._sample_points = sample_points
                        
                size_t point_count = plane.sample_points.size();
                Points points{ point_count, 3 };

                for (size_t i = 0; i < point_count; ++i)
                {
                    points.row(i)(0) = plane.sample_points[i][0];
                    points.row(i)(1) = plane.sample_points[i][1];
                    points.row(i)(2) = plane.sample_points[i][2];
                }

                // viewer.add_point_cloud(points, std::min(plane.bbox->color_.x() + 0.2f, 1.0f), plane.bbox->color_.y(), plane.bbox->color_.z());
                redraw_meshes = true;
			}
        }

        if (ImGui::Button("CGAL: Triangulated Surface Mesh Skeletonization"))
        {
            // CGAL::extract_mean_curvature_flow_skeleton();

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud) == -1) // Load point cloud
            {
                PCL_ERROR("Couldn't read file input.pcd \n");
                return (-1);
            }

            // Normal estimation (optional but recommended)
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);
            n.setInputCloud(cloud);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.compute(*normals);

            // Combine points and normals
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

            // Create search tree
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);

            // Poisson reconstruction
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth(8); // Set parameters as required
            poisson.setInputCloud(cloud_with_normals);
            pcl::PolygonMesh mesh;
            poisson.reconstruct(mesh);

            pcl::io::savePLYFile("mesh.ply", mesh);
        }

        if (ImGui::Button("Optimize local paths"))
        {
			for (int i = 0; i < detected_planes.size(); ++i)
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

				// Optimize graph
				// blast::Ant_colony_optimizer ant_colony_optimizer{&graph, 8, 100};
                // std::vector<size_t> path = ant_colony_optimizer.execute_iterations(100);
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

                redraw_meshes = true;
			}
        }

        // Optimize global paths
        if (ImGui::Button("Optimize global paths"))
        {
            blast::Graph graph;

            for (int i = 0; i < detected_planes.size(); ++i)
            {
				graph.add_node(std::make_shared<blast::Node>(std::format("P{}", i)));
            }

			for (int i = 0; i < detected_planes.size(); ++i)
			{
				for (int j = 0; j < detected_planes.size(); ++j)
				{
					if (i != j && !graph.exists_edge(i, j))
					{
						auto& plane1 = detected_planes[i];
						auto& plane2 = detected_planes[j];
						auto distance = (plane1.bbox->get_center() - plane2.bbox->get_center()).norm();
						graph.add_undirected_edge(i, j, distance);
					}
				}
			}

			// Optimize graph
			blast::Greedy_optimizer greedy_optimizer{ &graph };


			// Draw path
			auto path = greedy_optimizer.execute();
			for (int i = 0; i < path.size() - 1; ++i)
			{
				auto& plane1 = detected_planes[path[i]];
				auto& plane2 = detected_planes[path[i + 1]];

				// Add sphere for point1
				viewer.add_sphere(
					"path",
					plane1.bbox->get_center().cast<float>(),
					2.0f, Eigen::Vector3f(1.0f, 0.0f, 0.0f)
				);

				auto start_point = plane1.bbox->get_center().cast<float>();
				auto end_point = plane2.bbox->get_center().cast<float>();

				std::vector<Eigen::Vector3f> inbetween_points;

				//while (is_overlapping(p1, inbetween_points, p2))
				//{
				// Add more points
                inbetween_points.push_back(start_point + (end_point - start_point) * 0.25f + 8.0f * plane1.normal);
				inbetween_points.push_back(start_point + (end_point - start_point) * 0.75f + 8.0f * plane2.normal);
				//}

                std::vector<Eigen::Vector3f> all_points;
                all_points.push_back(start_point);

                // All points from inbetween_points
				for (auto& point : inbetween_points)
				{
					all_points.push_back(point);
				}

				all_points.push_back(end_point);


				// Draw lines in viewer
				for (int i = 0; i < all_points.size() - 1; ++i)
				{
					viewer.add_line(all_points[i], all_points[i + 1], Eigen::Vector3f(1.0, 0.0, 0.0));
                    viewer.add_sphere(
                        "path",
                        all_points[i],
                        0.4f, Eigen::Vector3f(1.0f, 0.0f, 0.0f)
                    );
				}
			}

			redraw_meshes = true;
        }

        if (ImGui::Button("Combine paths"))
        {

	        
        }

        ImGui::End();

        return redraw_meshes;  // True to update all meshes and camera
    };

    viewer.show();
}
