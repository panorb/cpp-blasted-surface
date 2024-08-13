#include <meshview/meshview.hpp>
#include <meshview/meshview_imgui.hpp>
#include <Eigen/Core>
#include "Eigen/Geometry"
#include <blast/point_cloud.hpp>

using namespace meshview;

int main(int argc, char** argv)
{
	Viewer viewer;
	viewer.draw_axes = true;
	viewer.camera.dist_to_center = 5.f;
    viewer.cull_face = false;


    std::string filename = R"(cube.ply)";
    std::unique_ptr<blast::Point_cloud> point_cloud = nullptr;

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

        ImGui::InputText("input text", &filename);
        if (ImGui::Button("Load point cloud"))
        {
			if (!filename.empty())
			{
                point_cloud = blast::Point_cloud::load_ply_file(filename);

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

			std::vector<glm::vec3> points = point_cloud->get_points();
            std::vector<glm::vec3> normals = point_cloud->estimate_normals(25.f);
            auto triangles_vec = point_cloud->greedy_triangulation(normals);

            Points vertices{points.size(), 3};
            for (size_t i = 0; i < points.size(); ++i)
			{
                vertices.row(i)(0) = points[i].x;
                vertices.row(i)(1) = points[i].y;
            	vertices.row(i)(2) = points[i].z;
			}

            Triangles triangles{triangles_vec.size(), 3};
            for (size_t i = 0; i < triangles_vec.size(); ++i)
            {
                triangles.row(i)(0) = triangles_vec[i][0];
                triangles.row(i)(1) = triangles_vec[i][1];
            	triangles.row(i)(2) = triangles_vec[i][2];
            }

            Points vw_normals = Eigen::Map<Points>(reinterpret_cast<float*>(normals.data()), normals.size(), 3);
            // vw_normals *= -1.0f;

            auto& pyra2 = viewer
                .add_mesh(vertices,
                    triangles,
                    0.f, 1.0f, 1.0f,
                    vw_normals)
                .set_shininess(32.f);
                //.translate(Vector3f(3.f, 3.f, 0.f));


			// blast::Point_cloud::greedy_triangulation(points, normals, vertices, triangles);

            
			redraw_meshes = true;
	        
        }
            
        ImGui::End();

        return redraw_meshes;  // True to update all meshes and camera
    };

    viewer.show();
}