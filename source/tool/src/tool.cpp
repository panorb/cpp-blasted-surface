#include "blast/tool.hpp"

#include <filesystem>
#include <GLFW/glfw3.h>

#include "blast/detected_plane_segment.hpp"
#include "blast/point_cloud.hpp"
#include "blast/planes/oliveira_planes.hpp"
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>

#include "pipeline.hpp"
#include "blast/greedy_optimizer.hpp"
#include "blast/sampler/grid_sampler.hpp"


namespace blast {

    void Tool::show()
    {
        viewer.show();
    }

    // Custom code
void Tool::on_open()
{/**/}

void Tool::on_close()
{/**/}

bool Tool::on_loop()
{
	/**/
    return false;
}

std::vector<std::string> example_files;
std::string selected_example_file = "knochen-komplett.pcd";
std::deque<Detected_plane_segment> detected_planes;
std::unique_ptr<blast::Point_cloud> base_point_cloud = nullptr;
size_t base_point_cloud_index = 0;
Oliveira_plane_segmenter oliveira_planes;

std::vector<Eigen::Vector3f> skeleton_vertices;

std::vector<Eigen::Vector3f> total_path;

Eigen::Vector3f line1_start = { 0.0f, 0.0f, 0.0f };
Eigen::Vector3f line1_end = { 0.0f, 110.0f, 0.0f };
Eigen::Vector3f line2_start = { -420.0f, -745.0f, 565.0f };
Eigen::Vector3f line2_end = { -342.22f, -822.78, 565.0f };
Eigen::Vector3f scanner_offset = { 0.0f, 0.0f, 0.0f };

Eigen::Matrix4f calculate_transformation_matrix(Eigen::Vector3f line1_start, Eigen::Vector3f line1_end, Eigen::Vector3f line2_start, Eigen::Vector3f line2_end, Eigen::Vector3f scanner_offset) {

    // This will move line 1s start to coincide with line 2
	Eigen::Vector3f start_transdlation = line2_start - line1_start;

    Eigen::Vector3f line1_direction = line1_end - line1_start;
	Eigen::Vector3f line2_direction = line2_end - line2_start;

    spdlog::info("Length of line 1: {0:.2f}", line1_direction.norm());
    spdlog::info("Length of line 2: {0:.2f}", line2_direction.norm());

	float scaling_factor = line2_direction.norm() / line1_direction.norm();

    Eigen::Vector3f line1_direction_normalized = line1_direction.normalized();
	Eigen::Vector3f line2_direction_normalized = line2_direction.normalized();

    // Compute rotation axis
	Eigen::Vector3f rotation_axis = line1_direction_normalized.cross(line2_direction_normalized);

    // Compute the rotation angle between the two direction vectors using the dot product
	float rotation_angle = std::acos(line1_direction_normalized.dot(line2_direction_normalized));

	// Compute the rotation matrix R (Rodrigues' rotation formula)
	Eigen::Matrix3f R = Eigen::AngleAxisf(rotation_angle, rotation_axis).toRotationMatrix();

	// Compute full transformation matrix (including rotation & translation)
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
	T.block<3, 3>(0, 0) = scaling_factor * R;
	T.block<3, 1>(0, 3) = start_transdlation;

	return T;
}

bool Tool::on_gui()
{
	/**/

	bool redraw_meshes = false;

    ImGui::Begin("Tool");

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
            Points points{ point_count, 3 };

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = base_point_cloud->get_points()[i][0];
                points.row(i)(1) = base_point_cloud->get_points()[i][1];
                points.row(i)(2) = base_point_cloud->get_points()[i][2];
            }

            viewer.add_point_cloud("base", points, 0.f, 1.0f, 1.0f);
            redraw_meshes = true;
        }

    }

    ImGui::SeparatorText("Oliveira Plane Segmentation");

    oliveira_planes.render_controls();
    if (ImGui::Button("Oliveira: Extract Planes"))
    {
        std::vector<Eigen::Vector3d> points = base_point_cloud->get_points();
        std::vector<Eigen::Vector3d> normals = base_point_cloud->estimate_normals(25.f);

        oliveira_planes.from_arrays(points, normals);

        auto planes = oliveira_planes.execute();

        viewer.delete_all("base");

        // Visualize planes
        for (int i = 0; i < planes.size(); ++i)
        {
            auto& plane = planes[i];
            Eigen::Vector3f normal = plane->R_ * Eigen::Vector3f(0, 0, plane->extent_(2));
            normal.normalize();

            // Add to detected planes vector
            auto detected_plane = Detected_plane_segment(plane, normal);
            detected_planes.push_back(detected_plane);

            auto box_points = plane->get_box_points();
            Points vertices{ box_points.size(), 3 };

            for (size_t i = 0; i < box_points.size(); ++i)
            {
                vertices.row(i)(0) = box_points[i].x();
                vertices.row(i)(1) = box_points[i].y();
                vertices.row(i)(2) = box_points[i].z();
            }

            Triangles triangles{ 12, 3 };
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

            auto indices = plane->get_point_indices_within_bounding_box(base_point_cloud->get_points_f());

			Points points{ indices.size(), 3 };

			for (size_t i = 0; i < indices.size(); ++i)
			{
				points.row(i)(0) = base_point_cloud->get_points()[indices[i]][0];
				points.row(i)(1) = base_point_cloud->get_points()[indices[i]][1];
				points.row(i)(2) = base_point_cloud->get_points()[indices[i]][2];
			}

			viewer.add_point_cloud("plane_" + std::to_string(i), points, r, g, b);

            //viewer.add_mesh(detected_plane.get_uuid(), vertices, triangles, r, g, b);
            // viewer.add_line(plane->get_center(), plane->get_center() + (2 * normal), Eigen::Vector3f(0.0, 1.0, 0.0));
        }

        redraw_meshes = true;
    }

    if (ImGui::Button("Remove last plane"))
    {
        if (detected_planes.begin() != detected_planes.end())
        {
			auto detected_plane = detected_planes.front();
	        //detected_plane.bbox->color_ = Eigen::Vector3f(1.0, 0.0, 0.0);

	        viewer.delete_all(detected_plane.get_uuid());
	        detected_planes.pop_front();

        } else
        {
            spdlog::warn("No planes to remove. Reached end of array...");
        }

        redraw_meshes = true;
    }

    if (ImGui::Button("CGAL: Triangulated Surface Mesh Skeletonization"))
    {
        // CGAL::extract_mean_curvature_flow_skeleton();
        skeleton_vertices = run_pipeline("./examples/" + selected_example_file);

		//Points skeleton_points{ skeleton_vertices.size(), 3 };

		for (size_t i = 0; i < skeleton_vertices.size(); ++i)
		{
			viewer.add_sphere(
				"sph",
				skeleton_vertices[i].cast<float>(),
				2.0f,
				Eigen::Vector3f(1.0, 1.0, 1.0)
			);
		}

		//viewer.add_point_cloud(skeleton_points, 1.0f, 1.0f, 1.0f);
        redraw_meshes = true;
        
    }

    if (ImGui::Button("Calculate normals based on skeleton vertices"))
    {
        // Correct normals of planes
        for (auto& plane : detected_planes)
        {
            // Save distance of nearest point in skeleton to plane
            Eigen::Vector3f nearest_point = skeleton_vertices[0];
            double min_distance = std::numeric_limits<double>::max();

            // Find nearest point in skeleton to plane
            for (Eigen::Vector3f candidate_point : skeleton_vertices) {
                Eigen::Vector3f center = plane.bbox->get_center();

                double distance = (candidate_point - center).norm();

                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_point = candidate_point;
                }
            }

            // Calculate which normal to use
            Eigen::Vector3f normal = plane.bbox->R_ * Eigen::Vector3f(0, 0, 1);

            // Calculate wether normal needs to be flipped to point away from the skeleton

            Eigen::Vector3f skeleton_to_plane = plane.bbox->get_center() - nearest_point;

            if (skeleton_to_plane.dot(normal) < 0)
            {
                normal = -normal;
            }

            plane.normal = normal;

            // Draw normal
            viewer.add_line(plane.bbox->get_center(), plane.bbox->get_center() + (10 * normal), Eigen::Vector3f(1.0, 0.0, 0.0));
        }

        redraw_meshes = true;
    }

    if (ImGui::Button("Generate sample points"))
    {
        for (auto& plane : detected_planes)
        {
            if (!plane.selected) continue;

            plane.sample_points = sample_points_on_grid(plane, 10.0, 0.0);
            // plane._sample_points = sample_points

            size_t point_count = plane.sample_points.size();
            Points points{ point_count, 3 };

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = plane.sample_points[i][0];
                points.row(i)(1) = plane.sample_points[i][1];
                points.row(i)(2) = plane.sample_points[i][2];
            }

            viewer.add_point_cloud("sample_points", points, 1.0f, 0.0f, 0.0f);
        }
        redraw_meshes = true;
    }

	if (ImGui::Button("Move sample points along normals"))
	{
		viewer.delete_all("sample_points");

        for (auto& plane : detected_planes)
        {
            size_t point_count = plane.sample_points.size();
            Points points{ point_count, 3 };

            for (auto& sample_point : plane.sample_points)
            {
                sample_point += plane.normal * 10.0f;
            }

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = plane.sample_points[i][0];
                points.row(i)(1) = plane.sample_points[i][1];
                points.row(i)(2) = plane.sample_points[i][2];
            }

            viewer.add_point_cloud("sample_points", points, 1.0f, 0.0f, 0.0f);
        }
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
            blast::Greedy_optimizer greedy_optimizer{ &graph, plane.sample_points };
            std::vector<size_t> path = greedy_optimizer.execute();
			plane.local_path = path;

            // Prepare outputting resulting path index sequence e.g. 0 -> 1 -> 2 -> 3
            /* std::string r = "";

            for (auto& node : path)
            {
                r += std::format("{0:d} -> ", node);
            }

            // Remove last arrow from output string
            r = r.substr(0, r.size() - 4);
            spdlog::info("Path: {0}", r); */

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
		std::vector<Eigen::Vector3f> all_centers;

        for (int i = 0; i < detected_planes.size(); ++i)
        {
            graph.add_node(std::make_shared<blast::Node>(std::format("P{}", i)));
			all_centers.push_back(detected_planes[i].bbox->get_center().cast<float>());
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
        blast::Greedy_optimizer greedy_optimizer{ &graph, all_centers };

        // total_path.insert(total_path.end(),detected_planes[0].local_path.begin(), detected_planes[0].local_path.begin());
        for (size_t idx : detected_planes[0].local_path)
        {
            total_path.push_back(detected_planes[0].sample_points[idx]);
        }

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

            auto start_point = plane1.sample_points[plane1.local_path.back()];
            auto end_point = plane2.sample_points[plane2.local_path.front()];

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

			total_path.insert(total_path.end(), all_points.begin(), all_points.end());

			for (size_t idx : plane2.local_path)
			{
				total_path.push_back(plane2.sample_points[idx]);
			}


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


    ImGui::SeparatorText("Koordinatentransformation");
    ImGui::Text("Linie in Punktwolke");
	ImGui::DragFloat3("Start", line1_start.data(), 0.1f);
    ImGui::DragFloat3("End", line1_end.data(), 0.1f);

    ImGui::Text("Linie in Roboter-KOS");
    ImGui::DragFloat3("Start", line2_start.data(), 0.1f);
    ImGui::DragFloat3("End", line2_end.data(), 0.1f);
    ImGui::DragFloat3("Scanner-Offset", scanner_offset.data(), 0.1f);

    if (ImGui::Button("Ausgabe in G-Code"))
    {
        Eigen::Matrix4f transformation_matrix = calculate_transformation_matrix(line1_start, line1_end, line2_start, line2_end, scanner_offset);

        spdlog::info("G-Code:");
        for (auto pt : total_path)
        {
            Eigen::Vector3f transformed = (transformation_matrix * pt.homogeneous()).hnormalized();
            spdlog::info("G01 X={0:.2f} Y={1:.2f} Z={2:.2f} A=-0.0 B=-0.0 C=45.0 F=200.0", transformed.x(), transformed.y(), transformed.z());
        }
    }

	ImGui::End();
    return redraw_meshes;
}

bool Tool::on_key(int key, input::Action action, int mods)
{
	/**/
    return true;
}

bool Tool::on_mouse_button(int key, input::Action action, int mods)
{
    if (key == GLFW_MOUSE_BUTTON_RIGHT)
    {
        // Shoot raycast from mouse position

        // Calculate the ray origin
        Eigen::Vector3f ray_origin = viewer.camera.get_pos();

        // Calculate the ray endpoint based on the mouse coordinates
        // Assuming the mouse coordinates are in normalized device coordinates (NDC)
        // where (-1, -1) is the bottom-left corner and (1, 1) is the top-right corner of the screen
        int width, height;
        glfwGetWindowSize((GLFWwindow*)viewer._window, &width, &height);

        float ndc_x = static_cast<float>(viewer._mouse_x) / width * 2 - 1;
        float ndc_y = 1.0f - (static_cast<float>(viewer._mouse_y) / height) * 2;

        spdlog::debug("ndc_x: {}, ndc_y: {}", ndc_x, ndc_y);

        // Calculate the ray direction by subtracting the ray origin from the ray endpoint
        Eigen::Vector3f ray_direction = viewer.camera.raycast(ndc_x, ndc_y); // (ray_endpoint - ray_origin).normalized();

        // Find the intersection point of the ray with the planes
        if (!detected_planes.empty())
        {
	        float selected_intersection_distance = std::numeric_limits<float>::max();
	        Detected_plane_segment selected_plane_segment = detected_planes[0];

	        for (auto& plane : detected_planes)
	        {
	            float intersection_distance = 0.0f;
	            plane.intersect_ray(ray_origin, ray_direction, intersection_distance);

	            if (intersection_distance < selected_intersection_distance)
	            {
	                selected_intersection_distance = intersection_distance;
	                selected_plane_segment = plane;
	            }
	        }
	        viewer.add_line(ray_origin, selected_plane_segment.bbox->get_center(), Eigen::Vector3f(0, 0, 1));
        }

        //spdlog::info("Deleting {}...", selected_plane_segment.get_uuid());
        // viewer.get_by_tag(selected_plane_segment.get_uuid())->enabled = false;

        // Visualize the ray
        viewer.add_line(ray_origin, ray_origin + ray_direction * 10000, Eigen::Vector3f(1, 0, 0));
        

        return false;
    }

    return true;
}

bool Tool::on_mouse_move(double x, double y)
{
	/**/
    return true;
}

bool Tool::on_scroll(double xoffset, double yoffset)
{
	/**/
    return true;
}
}  // namespace blast
