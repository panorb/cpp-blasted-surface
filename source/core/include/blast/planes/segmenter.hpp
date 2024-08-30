#pragma once

#define NOMINMAX
#define _USE_MATH_DEFINES
#include <imgui.h>
#include "segmenter_utility.hpp"
#include "bounding_box.hpp"

class Plane_segmenter {
public:
	Plane_segmenter() {};
	Plane_segmenter(SPointCloud::Ptr cloud, SNormalCloud::Ptr normals) : point_cloud_(cloud), normal_cloud_(normals) {}
	
	virtual const std::vector<std::shared_ptr<Oriented_bounding_box>> execute() = 0;
	virtual void render_controls() = 0;

	SPointCloud::Ptr point_cloud_;
	SNormalCloud::Ptr normal_cloud_;
};

