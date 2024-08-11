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


    std::string filename = "C:/example.ply";
    std::unique_ptr<blast::Point_cloud> point_cloud = nullptr;

    viewer.on_gui = [&]() -> bool {
        bool redraw_meshes = false;

        ImGui::ShowDemoWindow();

    	ImGui::SetNextWindowSize(ImVec2(400, 800), ImGuiCond_Once);

        ImGui::Begin("Editor");

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

            
        ImGui::End();

        return redraw_meshes;  // True to update all meshes and camera
    };

    viewer.show();
}
