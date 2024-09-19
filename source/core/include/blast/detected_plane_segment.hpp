#pragma once
#include "planes/bounding_box.hpp"

struct Detected_plane_segment
{
	std::shared_ptr<Oriented_bounding_box> bbox;
	Eigen::Vector3f normal;
	std::vector<Eigen::Vector3f> sample_points;

	Detected_plane_segment(std::shared_ptr<Oriented_bounding_box> bbox, Eigen::Vector3f normal, std::vector<Eigen::Vector3f> sample_points)
		: bbox(bbox), normal(normal), sample_points(sample_points)
	{
	}
};
