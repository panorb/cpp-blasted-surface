#pragma once

#define NOMINMAX
#define _USE_MATH_DEFINES
#include <imgui.h>
#include "segmenter_utility.hpp"
#include "bounding_box.hpp"

class PlaneSegmenter {
public:
	PlaneSegmenter() {};
	PlaneSegmenter(SPointCloud::Ptr cloud, SNormalCloud::Ptr normals) : point_cloud_(cloud), normal_cloud_(normals) {}
	
	virtual const std::vector<std::shared_ptr<OrientedBoundingBox>> execute() = 0;
	virtual void renderControls() = 0;

	SPointCloud::Ptr point_cloud_;
	SNormalCloud::Ptr normal_cloud_;
};

