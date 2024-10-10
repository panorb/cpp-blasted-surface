#pragma once
#include "meshview/meshview.hpp"


class Tool
{
public:
	Tool();
	void run();

	meshview::Viewer _viewer{};

	std::string pointcloud_file_path;
	std::vector<std::string> example_files;

	// std::function<bool()> l;
};

bool draw_imgui_gui(Tool& tool);