#pragma once

#include "segmenter_utility.hpp"
#include "bounding_box.hpp"
#include "planar_patch.hpp"

///     Araújo and Oliveira, “A robust statistics approach for plane
///     detection in unorganized point clouds,” Pattern Recognition, 2020.
class Plane_detector {
public:
	Plane_detector(float normal_similarity,
		float coplanarity,
		float outlier_ratio,
		float plane_edge_length)
		: patch_(std::make_shared<Planar_patch>()),
		normal_similarity_thr_(normal_similarity),
		coplanarity_thr_(coplanarity),
		outlier_ratio_thr_(outlier_ratio),
		plane_edge_length_thr_(plane_edge_length) {}
	~Plane_detector() = default;

	bool detect_from_point_cloud(const SPointCloud* point_cloud, const SNormalCloud* normal_cloud,
		const std::vector<size_t>& indices);

	std::shared_ptr<Oriented_bounding_box> delimit_plane();

	/// \param idx  Index of point in point_cloud_
	bool is_inlier(size_t idx);

	bool has_visited(size_t idx);

	void mark_visited(size_t idx);

	void add_point(size_t idx);

	void update();

	bool is_false_positive();

public:
	std::shared_ptr<Planar_patch> patch_;
	const SPointCloud* point_cloud_;
	const SNormalCloud* normal_cloud_;
	std::vector<size_t> indices_;

	float min_normal_diff_;
	float max_point_dist_;

	bool stable_ = false;

	size_t num_new_points_ = 0;
	size_t num_updates_ = 0;

	size_t index_;

	pcl::Vertices get_plane_perimeter_vertices();

private:
	Eigen::Vector3f min_bound_;
	Eigen::Vector3f max_bound_;
	float longest_edge_;

	Eigen::Matrix3f B_;

	float normal_similarity_thr_;
	float coplanarity_thr_;
	float outlier_ratio_thr_;
	float plane_edge_length_thr_;

	std::unordered_set<size_t> visited_indices_;

	struct RotatedRect {
		Eigen::Matrix3f B;
		Eigen::Matrix3Xf M;
		float area;
		Eigen::Vector3f bottom_left;
		Eigen::Vector3f top_right;

		RotatedRect(const Eigen::Matrix3Xf& M,
			const Eigen::Matrix3f& B,
			float degrees);
	};

	void estimate_plane();

	bool robust_planarity_test();

	inline bool is_normal_valid() const;

	bool is_distance_valid();

	/// \param M  3D perimeter points
	void get_plane_perimeter_points(Eigen::Matrix3Xf& M);

	void construct_orthogonal_basis(Eigen::Matrix3f& B);
};

using PlaneDetectorPtr = std::shared_ptr<Plane_detector>;
