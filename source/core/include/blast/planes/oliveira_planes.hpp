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

class Oliveira_plane_segmenter : public Plane_segmenter {
public:
	Oliveira_plane_segmenter() {}
	Oliveira_plane_segmenter(
		SPointCloud::Ptr cloud,
		SNormalCloud::Ptr normals,
		float normal_similarity_deg = 60.f,
		float coplanarity_deg = 75,
		float outlier_ratio = 0.75,
		float min_plane_edge_length = 0.0,
		size_t min_num_points = 0) : normal_similarity_deg_(normal_similarity_deg), coplanarity_deg_(coplanarity_deg), outlier_ratio_(outlier_ratio), min_plane_edge_length_(min_plane_edge_length), min_num_points_(min_num_points), Plane_segmenter(cloud, normals) {
	}
	void from_arrays(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals);
	virtual const std::vector<std::shared_ptr<Oriented_bounding_box>> execute();
	virtual void render_controls();

	float normal_similarity_deg_ = 60.f;
	float coplanarity_deg_ = 75;
	float outlier_ratio_ = 0.75f;
	float min_plane_edge_length_ = 0.0f;
	int min_num_points_ = 0;
private:
	const std::vector<std::shared_ptr<Oriented_bounding_box>> detect_planar_patches();
	bool split_and_detect_planes_recursive(
		const BoundaryVolumeHierarchyPtr& node,
		std::vector<PlaneDetectorPtr>& planes,
		std::vector<PlaneDetectorPtr>& plane_points, float real_min_plane_edge_length, int real_min_num_points);
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
		std::vector<std::shared_ptr<Oriented_bounding_box>>& patches);
};
