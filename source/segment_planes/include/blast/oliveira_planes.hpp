#pragma once

#include "segmenter.hpp"
#include "bounding_box.hpp"
#include "boundary_volume_hierarchy.hpp"
#include "plane_detector.hpp"
#include "disjoint_set.hpp"

#include <queue>
#include <mutex>
#include <chrono>
#include <glm/vec3.hpp>

class OliveiraPlaneSegmenter : public PlaneSegmenter {
public:
	OliveiraPlaneSegmenter() {}
	OliveiraPlaneSegmenter(
		SPointCloud::Ptr cloud,
		SNormalCloud::Ptr normals,
		float normal_similarity_deg = 60.f,
		float coplanarity_deg = 75,
		float outlier_ratio = 0.75,
		float min_plane_edge_length = 0.0,
		size_t min_num_points = 0) : normal_similarity_deg_(normal_similarity_deg), coplanarity_deg_(coplanarity_deg), outlier_ratio_(outlier_ratio), min_plane_edge_length_(min_plane_edge_length), min_num_points_(min_num_points), PlaneSegmenter(cloud, normals) {
	}
	void fromArrays(std::vector<glm::vec3> points, std::vector<glm::vec3> normals);
	virtual const std::vector<std::shared_ptr<OrientedBoundingBox>> execute();
	virtual void renderControls();
private:
	const std::vector<std::shared_ptr<OrientedBoundingBox>> detect_planar_patches();
	bool split_and_detect_planes_recursive(
		const BoundaryVolumeHierarchyPtr& node,
		std::vector<PlaneDetectorPtr>& planes,
		std::vector<PlaneDetectorPtr>& plane_points);
	void grow(
		std::vector<PlaneDetectorPtr>& planes,
		std::vector<PlaneDetectorPtr>& plane_points,
		const std::vector<std::vector<int>>& neighbors);
	void merge(
		std::vector<PlaneDetectorPtr>& planes,
		std::vector<PlaneDetectorPtr>& plane_points,
		const std::vector<std::vector<int>>& neighbors);
	bool update(
		std::vector<PlaneDetectorPtr>& planes
	);
	void extract_patches_from_planes(
		const std::vector<PlaneDetectorPtr>& planes,
		std::vector<std::shared_ptr<OrientedBoundingBox>>& patches);

	float normal_similarity_deg_ = 60.f;
	float coplanarity_deg_ = 75;
	float outlier_ratio_ = 0.75f;
	float min_plane_edge_length_ = 0.0f;
	int min_num_points_ = 0;
};
