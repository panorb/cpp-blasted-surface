#include <meshview/meshview.hpp>
#include <meshview/meshview_imgui.hpp>
#include <Eigen/Core>
#include "Eigen/Geometry"

// Eigene Includes
#include <blast/point_cloud.hpp>
//#include <blast/oliveira_planes.hpp>
#include <blast/voxel_grid.hpp>
#include <spdlog/spdlog.h>
// #include <spdlog/spdlog.h>

using namespace meshview;

int main(int argc, char** argv)
{
	Viewer viewer;
	viewer.draw_axes = true;
	viewer.camera.dist_to_center = 5.f;
    viewer.cull_face = false;

    std::string filename = R"(cube.ply)";
    std::unique_ptr<blast::Point_cloud> point_cloud = nullptr;
    //OliveiraPlaneSegmenter oliveira_planes;

	std::shared_ptr<blast::Voxel_grid> voxel_grid;

    std::vector<Mesh*> voxel_grid_meshes;
    PointCloud* voxel_grid_points;
    

    viewer.on_gui = [&]() -> bool {
        bool redraw_meshes = false;

        ImGui::ShowDemoWindow();

    	ImGui::SetNextWindowSize(ImVec2(400, 800), ImGuiCond_Once);

        ImGui::Begin("Editor");

        if (ImGui::Button("Clear all"))
        {
	        viewer.meshes.clear();
            viewer.point_clouds.clear();
            point_cloud = nullptr;
        }

        ImGui::InputText("Pointcloud file", &filename);
        if (ImGui::Button("Load point cloud"))
        {
			if (!filename.empty())
			{
                if (filename.ends_with(".ply"))
                {
                    point_cloud = blast::Point_cloud::load_ply_file(filename);
				}
				else if (filename.ends_with(".pcd"))
				{
					point_cloud = blast::Point_cloud::load_pcd_file(filename);
				}
				else
				{
					//spdlog::error("Unsupported file format");
                    return false;
				}

                size_t point_count = point_cloud->get_points().size();
                Points points{point_count, 3};

                for (size_t i = 0; i < point_count; ++i)
				{
					points.row(i)(0) = point_cloud->get_points()[i][0];
                    points.row(i)(1) = point_cloud->get_points()[i][1];
                    points.row(i)(2) = point_cloud->get_points()[i][2];
				}

                viewer.add_point_cloud(points, 0.f, 1.0f, 1.0f);
                redraw_meshes = true;
			}
	        
        }

        if (ImGui::Button("Downsample using voxel grid")) {
            point_cloud->voxelgrid_downsample(0.04f);

            viewer.point_clouds.clear();

            size_t point_count = point_cloud->get_points().size();
            Points points{ point_count, 3 };

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = point_cloud->get_points()[i][0];
                points.row(i)(1) = point_cloud->get_points()[i][1];
                points.row(i)(2) = point_cloud->get_points()[i][2];
            }

            viewer.add_point_cloud(points, 0.f, 1.0f, 1.0f);

            redraw_meshes = true;
        }


        if (ImGui::Button("Greedy Triangulation"))
        {
            
			viewer.meshes.clear();

			std::vector<Eigen::Vector3d> points = point_cloud->get_points();
            std::vector<Eigen::Vector3d> p_normals = point_cloud->estimate_normals(25.f);
            auto triangles_vec = point_cloud->greedy_triangulation(p_normals);

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
                .add_mesh(vertices,
                    triangles,
                    0.f, 1.0f, 1.0f,
                    vertices)
                .set_shininess(32.f);
                //.translate(Vector3f(3.f, 3.f, 0.f));


			// blast::Point_cloud::greedy_triangulation(points, normals, vertices, triangles);

            
			redraw_meshes = true;
	        
        }

        ImGui::Separator();

        if (ImGui::Button("Create voxel grid based on point cloud"))
        {
            voxel_grid = blast::Voxel_grid::create_from_point_cloud(*point_cloud, 0.01);
            auto slightly_expanded_grid = blast::dilate_grid(*voxel_grid, 2);
            auto strongly_expanded_grid = blast::dilate_grid(*voxel_grid, 3);
            voxel_grid = blast::subtract_grids(*strongly_expanded_grid, *slightly_expanded_grid);

			auto voxels = voxel_grid->get_voxels();
			auto voxel_count = voxels.size();

        	Points points{ voxel_count, 3 };

            for (size_t i = 0; i < voxel_count; ++i)
            {
                auto voxel = voxels[i];
                spdlog::info("Voxel [{0:d}, {1:d}, {2:d}]", voxel.grid_index.x(), voxel.grid_index.y(), voxel.grid_index.z());

                Eigen::Vector3d voxel_center = voxel_grid->get_voxel_center_coordinate(voxel.grid_index);

                points.row(i)(0) = voxel_center.x();
                points.row(i)(1) = voxel_center.y();
                points.row(i)(2) = voxel_center.z();

				voxel_grid_meshes.push_back(
                    &viewer.add_cube(voxel_center.cast<float>(), 0.01)
                );
            }

            // viewer.add_cube();
            voxel_grid_points = &viewer.add_point_cloud(points, 0.f, 1.0f, 0.0f).enable(false);
            redraw_meshes = true;
        }

        if (ImGui::Button("Toggle visibility"))
        {
			for (auto& mesh : voxel_grid_meshes)
			{
                mesh->enable(!mesh->enabled);
			}

            voxel_grid_points->enable(!voxel_grid_points->enabled);
        }

        /*oliveira_planes.renderControls();
        if (ImGui::Button("Oliveira: Extract Planes"))
        {
            std::vector<glm::vec3> points = point_cloud->get_points();
            std::vector<glm::vec3> normals = point_cloud->estimate_normals(25.f);

            oliveira_planes.fromArrays(points, normals);

            auto planes = oliveira_planes.execute();
            // Visualize planes
            for (auto& plane : planes)
			{
				auto box_points = plane->GetBoxPoints();
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

				viewer.add_mesh(vertices, triangles, 1.0f, 0.0f, 0.0f);
			}
        }*/
            
        ImGui::End();

        return redraw_meshes;  // True to update all meshes and camera
    };

    viewer.show();
}
