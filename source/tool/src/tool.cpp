#include "tool.hpp"

#include <filesystem>
#include <imgui.h>

#include "pipeline.hpp"


Tool::Tool()
{
	_viewer.draw_axes = true;
	_viewer.camera.dist_to_center = 5.f;
	_viewer.cull_face = false;

    _viewer.on_gui = [this] { return draw_imgui_gui(*this); };
}

void Tool::run()
{
	_viewer.show();
}

bool draw_imgui_gui(Tool& t)
{
    ImGui::ShowDemoWindow();

    ImGui::SetNextWindowSize(ImVec2(400, 800), ImGuiCond_Once);

    ImGui::Begin("Tool");

    if (ImGui::BeginCombo("Pointcloud file", t.pointcloud_file_path.c_str()))
    {
        t.example_files.clear();
        // Get all files in directory "/examples"
        for (const auto& entry : std::filesystem::directory_iterator(R"(.\examples)"))
        {
            auto res = std::filesystem::path(entry.path()).filename().string();
            t.example_files.push_back(res);

            if (ImGui::Selectable(t.example_files.back().c_str(), t.example_files.back() == t.pointcloud_file_path))
            {
                t.pointcloud_file_path = t.example_files.back();
            }
        }

        ImGui::EndCombo();
    }

    if (ImGui::Button("Run pipeline"))
    {
        run_pipeline(".\\examples\\" + t.pointcloud_file_path);
    }

    ImGui::End();

    return true;
}