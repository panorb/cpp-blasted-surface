#include "blast/planes/oliveira_planes.hpp"

const std::vector<std::shared_ptr<Oriented_bounding_box>> Oliveira_plane_segmenter::execute() {
	return detect_planar_patches();
}

void Oliveira_plane_segmenter::render_controls()
{
	ImGui::DragFloat("Normal similiarity (deg)", &normal_similarity_deg_, .1, 1.0, 90.0);
	ImGui::DragFloat("Coplanarity (deg)", &coplanarity_deg_, .1, 1.0, 90.0);
	ImGui::DragFloat("Outlier ratio", &outlier_ratio_, .01, 0.0, 1.0);
	ImGui::InputFloat("Min plane edge length", &min_plane_edge_length_);
	ImGui::DragInt("Min num points", &min_num_points_, 1.0, 3, 200);
}

const std::vector<std::shared_ptr<Oriented_bounding_box>> Oliveira_plane_segmenter::detect_planar_patches() {
	spdlog::info("started detecting planar patches");
	// spdlog::info("started detecting planar patches");
	SPoint min_bound;
	SPoint max_bound;
	
	pcl::getMinMax3D(*point_cloud_, min_bound, max_bound);

	float real_min_plane_edge_length = min_plane_edge_length_;

	if (min_plane_edge_length_ <= 0) {
		real_min_plane_edge_length = 0.01 * (max_bound.getVector3fMap() - min_bound.getVector3fMap()).maxCoeff();
	}

	spdlog::info("min_bound: {0:f}, {1:f}, {2:f}", min_bound.x, min_bound.y, min_bound.z);
	spdlog::info("max_bound: {0:f}, {1:f}, {2:f}", max_bound.x, max_bound.y, max_bound.z);

	int real_min_num_points = min_num_points_;

	if (min_num_points_ == 0) {
		
		real_min_num_points = std::max(static_cast<size_t>(10),
			static_cast<size_t>(point_cloud_->points.size() * 0.001));
	}

	// geometry::KDTreeSearchParam search_param = geometry::KDTreeSearchParamHybrid{ 1, 100 },
	// geometry::KDTreeFlann kdtree;
	// kdtree.SetGeometry(point_cloud);

	pcl::KdTreeFLANN<SPoint> kdtree;
	kdtree.setInputCloud(point_cloud_);
	
	std::vector<std::vector<int>> neighbors;
	neighbors.resize(point_cloud_->points.size());

	auto start = high_resolution_clock::now();
	spdlog::info("searching ktree for neighbors");

	std::mutex num_mutex;
	int finished_threads = 0;
	size_t num_points = point_cloud_->points.size();

#pragma omp parallel for schedule(static)
	for (int i = 0; i < static_cast<int>(point_cloud_->points.size()); i++) {
		std::vector<float> distance2;
		kdtree.nearestKSearch(point_cloud_->points[i], 100, neighbors[i], distance2);
		
		num_mutex.lock();
		++finished_threads;

		if (finished_threads % 100 == 0) {
			spdlog::info("{}/{}", finished_threads, num_points);
		}
		num_mutex.unlock();

		// kdtree.SearchRadius(point_cloud.points_[i], 100, neighbors[i], distance2);
		// kdtree.radiusSearch(point_cloud.)
	}

	auto end = high_resolution_clock::now();
	auto duration = duration_cast<seconds>(end - start);

	spdlog::info("neighbors of {} points found in {} seconds", num_points, duration.count());

	// partition the point cloud and search for planar regions
	BoundaryVolumeHierarchyPtr root = std::make_shared<Boundary_volume_hierarchy>(
		point_cloud_.get(), normal_cloud_.get(), min_bound.getVector3fMap(), max_bound.getVector3fMap());
	std::vector<PlaneDetectorPtr> planes;
	std::vector<PlaneDetectorPtr> plane_points(point_cloud_->points.size(), nullptr);
	split_and_detect_planes_recursive(root, planes, plane_points, real_min_plane_edge_length, real_min_num_points);

	bool changed;
	do {
		spdlog::info("grow");
		grow(planes, plane_points, neighbors);

		spdlog::info("merge");
		merge(planes, plane_points, neighbors);

		changed = update(planes);
	} while (changed);

	// extract planar patches by calculating the bounds of each detected plane
	std::vector<std::shared_ptr<Oriented_bounding_box>> patches;
	extract_patches_from_planes(planes, patches);

	return patches;
}

bool Oliveira_plane_segmenter::split_and_detect_planes_recursive(
	const BoundaryVolumeHierarchyPtr& node,
	std::vector<PlaneDetectorPtr>& planes,
	std::vector<PlaneDetectorPtr>& plane_points,
	float real_min_plane_edge_length,
	int real_min_num_points
	) {
	spdlog::info("split_and_detect_planes_recursive");
	// Grad in Radian umrechnen
	const float normal_similarity_rad = normal_similarity_deg_ * M_PI / 180.0;
	const float coplanarity_rad = coplanarity_deg_ * M_PI / 180.0;

	// Cosinus anwenden, um Werte zwischen -1 und 1 zu erhalten
	const float normal_similiarity = cos(normal_similarity_rad);
	const float coplanarity = cos(coplanarity_rad);

	// Wenn weniger als die minmale Anzahl an Punkten in der Node
	// vorhanden sind, kann keine Ebene erzeugt werden
	if (node->indices_.size() < real_min_num_points) return false;

	bool node_has_plane = false;
	bool child_has_plane = false;

	node->partition();

	for (const auto& child : node->children_) {
		if (child != nullptr &&
			split_and_detect_planes_recursive(child, planes, plane_points, real_min_plane_edge_length, real_min_num_points)) {
			child_has_plane = true;
		}
	}

	if (!child_has_plane && node->level_ > 2) {
		auto plane = std::make_shared<Plane_detector>(normal_similiarity,
			coplanarity, outlier_ratio_,
			real_min_plane_edge_length);
		if (plane->detect_from_point_cloud(node->point_cloud_, node->normal_cloud_, node->indices_)) {
			node_has_plane = true;
			planes.push_back(plane);

			// assume ownership of these indices
			for (const size_t& idx : plane->indices_) {
				plane_points[idx] = plane;
			}
		}
	}

	return node_has_plane || child_has_plane;
}

void Oliveira_plane_segmenter::grow(
	std::vector<PlaneDetectorPtr>& planes,
	std::vector<PlaneDetectorPtr>& plane_points,
	const std::vector<std::vector<int>>& neighbors) {
	// Sort so that least noisy planes grow first
	std::sort(planes.begin(), planes.end(),
		[](const PlaneDetectorPtr& a, const PlaneDetectorPtr& b) {
			return a->min_normal_diff_ > b->min_normal_diff_;
		});

	std::queue<size_t> queue;
	for (auto&& plane : planes) {
		if (plane->stable_) continue;

		// Consider each neighbor of each point associated with this plane
		for (const size_t& idx : plane->indices_) {
			queue.push(idx);
		}

		while (!queue.empty()) {
			const size_t idx = queue.front();
			queue.pop();
			for (const size_t& nbr : neighbors[idx]) {
				// Skip if this neighboring point has been claimed, or,
				// if this plane has already visited.
				if (plane_points[nbr] != nullptr || plane->has_visited(nbr))
					continue;
				if (plane->is_inlier(nbr)) {
					// Add this point to the plane and claim ownership
					plane->add_point(nbr);
					plane_points[nbr] = plane;
					// Since the nbr point has been added to the plane,
					// be sure to consider *its* neighbors, too.
					queue.push(nbr);
				}
				else {
					// Nothing to be done with this neighbor point
					plane->mark_visited(nbr);
				}
			}
		}
	}
}

void Oliveira_plane_segmenter::merge(
	std::vector<PlaneDetectorPtr>& planes,
	std::vector<PlaneDetectorPtr>& plane_points,
	const std::vector<std::vector<int>>& neighbors) {
	const size_t n = planes.size();
	for (size_t i = 0; i < n; i++) {
		planes[i]->index_ = i;
	}

	std::vector<bool> graph(n * n, false);
	std::vector<bool> disconnected_planes(n * n, false);
	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			const Eigen::Vector3f& ni = planes[i]->patch_->normal_;
			const Eigen::Vector3f& nj = planes[j]->patch_->normal_;
			const float normal_thr = std::min(planes[i]->min_normal_diff_,
				planes[j]->min_normal_diff_);
			disconnected_planes[i * n + j] = std::abs(ni.dot(nj)) < normal_thr;
			disconnected_planes[j * n + i] = disconnected_planes[i * n + j];
		}
	}

	for (auto&& plane : planes) {
		const size_t i = plane->index_;
		for (const size_t& idx : plane->indices_) {
			for (const int& nbr : neighbors[idx]) {
				auto& nplane = plane_points[nbr];
				if (nplane == nullptr) continue;
				const size_t j = nplane->index_;

				if (nplane == plane || graph[i * n + j] || graph[j * n + i] ||
					disconnected_planes[i * n + j] || plane->has_visited(nbr) ||
					nplane->has_visited(idx))
					continue;

				plane->mark_visited(nbr);
				nplane->mark_visited(idx);

				const Eigen::Vector3f& pi = point_cloud_->points[idx].getVector3fMap();
				const Eigen::Vector3f& ni = normal_cloud_->points[idx].getNormalVector3fMap();
				const Eigen::Vector3f& pj = point_cloud_->points[nbr].getVector3fMap();
				const Eigen::Vector3f& nj = normal_cloud_->points[nbr].getNormalVector3fMap();
				const float dist_thr = std::max(plane->max_point_dist_,
					nplane->max_point_dist_);
				const float normal_thr = std::min(plane->min_normal_diff_,
					nplane->min_normal_diff_);

				graph[i * n + j] =
					std::abs(plane->patch_->normal_.dot(nj)) > normal_thr &&
					std::abs(nplane->patch_->normal_.dot(ni)) >
					normal_thr &&
					std::abs(plane->patch_->get_signed_distance_to_point(pj)) <
					dist_thr &&
					std::abs(nplane->patch_->get_signed_distance_to_point(pi)) <
					dist_thr;
			}
		}
	}

	Disjoint_set ds(n);
	for (size_t i = 0; i < n; i++) {
		for (size_t j = i + 1; j < n; j++) {
			if (graph[i * n + j] || graph[j * n + i]) {
				ds.set_union(i, j);
			}
		}
	}

	std::vector<size_t> largest_planes(n);
	std::iota(largest_planes.begin(), largest_planes.end(), 0);
	for (size_t i = 0; i < n; i++) {
		const size_t root = ds.find(i);
		if (planes[largest_planes[root]]->indices_.size() <
			planes[i]->indices_.size()) {
			largest_planes[root] = i;
		}
	}

	for (size_t i = 0; i < n; i++) {
		const size_t root = largest_planes[ds.find(i)];
		if (root == i) continue;
		for (const size_t& idx : planes[i]->indices_) {
			planes[root]->add_point(idx);
			plane_points[idx] = planes[root];
		}
		planes[root]->max_point_dist_ = std::max(planes[root]->max_point_dist_,
			planes[i]->max_point_dist_);
		planes[root]->min_normal_diff_ = std::min(
			planes[root]->min_normal_diff_, planes[i]->min_normal_diff_);
		planes[i].reset();
	}

	planes.erase(std::remove_if(planes.begin(), planes.end(),
		[](const PlaneDetectorPtr& plane) {
			return plane == nullptr;
		}),
		planes.end());
}

bool Oliveira_plane_segmenter::update(
	std::vector<PlaneDetectorPtr>& planes
) {
	bool changed = false;
	for (auto&& plane : planes) {
		const bool more_than_half_points_are_new =
			3 * plane->num_new_points_ > plane->indices_.size();
		if (more_than_half_points_are_new) {
			plane->update();
			plane->stable_ = false;
			changed = true;
		}
		else {
			plane->stable_ = true;
		}
	}
	return changed;
}

void Oliveira_plane_segmenter::from_arrays(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> normals)
{
	
	point_cloud_ = std::make_shared<SPointCloud>();
	normal_cloud_ = std::make_shared<SNormalCloud>();

	for (size_t i = 0; i < points.size(); i++)
	{
		SPoint p;
		p.x = points[i].x();
		p.y = points[i].y();
		p.z = points[i].z();
		point_cloud_->points.push_back(p);

		SNormal n;
		n.normal_x = normals[i].x();
		n.normal_y = normals[i].y();
		n.normal_z = normals[i].z();
		normal_cloud_->points.push_back(n);
	}
}

void Oliveira_plane_segmenter::extract_patches_from_planes(
	const std::vector<PlaneDetectorPtr>& planes,
	std::vector<std::shared_ptr<Oriented_bounding_box>>& patches) {
	// Colors (default MATLAB colors)
	static constexpr int NUM_COLORS = 1;
	static std::array<Eigen::Vector3f, NUM_COLORS> colors = {
			Eigen::Vector3f(1.0, 0.0, 0.0)
	};
	/*static std::array<Eigen::Vector3f, NUM_COLORS> colors = {
			Eigen::Vector3f(0.8500, 0.3250, 0.0980),
			Eigen::Vector3f(0.9290, 0.6940, 0.1250),
			Eigen::Vector3f(0.4940, 0.1840, 0.5560),
			Eigen::Vector3f(0.4660, 0.6740, 0.1880),
			Eigen::Vector3f(0.3010, 0.7450, 0.9330),
			Eigen::Vector3f(0.6350, 0.0780, 0.1840) };*/

	for (size_t i = 0; i < planes.size(); i++) {
		if (!planes[i]->is_false_positive()) {
			// create a patch by delimiting the plane using its perimeter points
			auto obox = planes[i]->delimit_plane();
			// planes[i]->
			// obox->color_ = colors[i % NUM_COLORS];
			patches.push_back(obox);
		}
	}
}
