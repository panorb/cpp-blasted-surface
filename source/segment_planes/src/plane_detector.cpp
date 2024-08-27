#include "blast/plane_detector.hpp"

/// \brief Estimate plane from point cloud and selected point indices.
///
/// \param point_cloud The point cloud (with points and normals).
/// \param indices Point indices within point cloud to use.
/// \return True if indices of point cloud pass robust planarity tests.
bool PlaneDetector::DetectFromPointCloud(const SPointCloud* point_cloud, const SNormalCloud* normal_cloud, const std::vector<size_t>& indices) {
    if (point_cloud->empty()) return false;

    // Hold a reference to the point cloud. This PlaneDetector
    // object shall be released before the PointCloud object.
    point_cloud_ = point_cloud;
    normal_cloud_ = normal_cloud;
    indices_ = indices;

    // TODO: check if there are enough points

    // estimate a plane from the relevant points
    EstimatePlane();

    // check that the estimated plane passes the robust planarity tests
    return RobustPlanarityTest();
}

/// \brief Delimit the plane using its perimeter points.
///
/// \return A patch which is the bounded version of the plane.
std::shared_ptr<OrientedBoundingBox> PlaneDetector::DelimitPlane() {
    Eigen::Matrix3Xf M;
    GetPlanePerimeterPoints(M);

    // Bisection search to find new rotated basis that
    // minimizes the area of bounded plane.
    double min_angled = 0;
    double max_angled = 90;
    static constexpr double ANGLE_RESOLUTION_DEG = 5;
    while (max_angled - min_angled > ANGLE_RESOLUTION_DEG) {
        const double mid = (max_angled + min_angled) / 2.;
        const double left = (min_angled + mid) / 2.;
        const double right = (max_angled + mid) / 2.;

        RotatedRect leftRect(M, B_, left);
        RotatedRect rightRect(M, B_, right);
        if (leftRect.area < rightRect.area) {
            max_angled = mid;
        }
        else {
            min_angled = mid;
        }
    }

    // Create the optimum basis found from bisection search
    const double theta = (min_angled + max_angled) / 2.;
    RotatedRect rect(M, B_, theta);

    // Update the center of the patch
    patch_->center_ -= rect.B.col(0).dot(patch_->center_) * rect.B.col(0);
    patch_->center_ -= rect.B.col(1).dot(patch_->center_) * rect.B.col(1);
    patch_->center_ +=
        (rect.bottom_left(0) + rect.top_right(0)) / 2. * rect.B.col(0);
    patch_->center_ +=
        (rect.bottom_left(1) + rect.top_right(1)) / 2. * rect.B.col(1);

    // Scale basis to fit points
    const double width = (rect.top_right.x() - rect.bottom_left.x());
    const double height = (rect.top_right.y() - rect.bottom_left.y());
    const double depth = (rect.top_right.z() - rect.bottom_left.z());

    std::shared_ptr<OrientedBoundingBox> obox =
        std::make_shared<OrientedBoundingBox>();
    obox->center_ = patch_->center_;
    obox->R_ = rect.B;
    obox->extent_ = Eigen::Vector3f(width, height, 0.001);
    return obox;
}

/// \brief Determine if a point is an inlier to the estimated plane model.
///
/// \param idx  Index of point in point_cloud_

bool PlaneDetector::IsInlier(size_t idx) {
    const Eigen::Vector3f& point = point_cloud_->points[idx].getVector3fMap();
    const Eigen::Vector3f& normal = normal_cloud_->points[idx].getNormalVector3fMap();
    const bool valid_normal =
        std::abs(patch_->normal_.dot(normal)) > min_normal_diff_;
    const bool valid_dist =
        std::abs(patch_->GetSignedDistanceToPoint(point)) <
        max_point_dist_;
    return valid_normal && valid_dist;
}

/// \brief Check if cloud point at index idx has been visited.

bool PlaneDetector::HasVisited(size_t idx) {
    return visited_indices_.find(idx) != visited_indices_.end();
}

/// \brief Mark the cloud point at index idx as visited.

void PlaneDetector::MarkVisited(size_t idx) { visited_indices_.insert(idx); }

/// \brief Include an addition point at index idx as part of plane set.

void PlaneDetector::AddPoint(size_t idx) {
    indices_.push_back(idx);
    num_new_points_++;
}

/// \brief Estimate the plane parameters again to include added points.

void PlaneDetector::Update() {
    EstimatePlane();
    visited_indices_.clear();
    num_new_points_ = 0;
    num_updates_++;
}

/// \brief Check if plane is considered a false positive

bool PlaneDetector::IsFalsePositive() {
    EstimatePlane();  // TODO: just recalc longest_edge?
    return num_updates_ == 0 || longest_edge_ < plane_edge_length_thr_;
}

/// \brief Estimate plane from point cloud and selected point indices.

void PlaneDetector::EstimatePlane() {
    min_bound_ =
        Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    max_bound_ =
        -Eigen::Vector3f::Constant(std::numeric_limits<float>::max());

    // Calculate the median of the points and normals to estimate plane.
    const size_t N = indices_.size();
    std::vector<float> center_buf(N, 0);
    std::vector<float> normal_buf(N, 0);
    for (size_t d = 0; d < 3; d++) {
        // put data into buffer
        for (size_t i = 0; i < N; i++) {
            center_buf[i] = point_cloud_->points[indices_[i]].getArray3fMap()(d);
            normal_buf[i] = normal_cloud_->points[indices_[i]].getNormalVector3fMap()(d);

            // Simultaneously calculate the bounds of the associated points
            min_bound_(d) = std::min(min_bound_(d),
                point_cloud_->points[indices_[i]].getArray3fMap()(d));
            max_bound_(d) = std::max(max_bound_(d),
                point_cloud_->points[indices_[i]].getArray3fMap()(d));
        }
        // compute the median along this dimension
        patch_->center_(d) = GetMedian(center_buf);
        patch_->normal_(d) = GetMedian(normal_buf);
    }
    patch_->normal_.normalize();

    // orthogonal distance to plane
    patch_->dist_from_origin_ = -patch_->normal_.dot(patch_->center_);

    longest_edge_ = (max_bound_ - min_bound_).maxCoeff();

    ConstructOrthogonalBasis(B_);
}

/// brief Use robust statistics (i.e., median) to test planarity.
///
/// Follows Sec 3.2 of [ArujoOliveira2020].
///
/// \return True if passes tests.

bool PlaneDetector::RobustPlanarityTest() {
    // Calculate statistics to robustly test planarity.
    const size_t N = indices_.size();
    // Stores point-to-plane distance of each associated point.
    std::vector<float> point_distances(N, 0);
    // Stores dot product (similarity) of each point normal to plane normal.
    std::vector<float> normal_similarities(N, 0);
    for (size_t i = 0; i < N; i++) {
        const Eigen::Vector3f& normal = normal_cloud_->points[indices_[i]].getNormalVector3fMap();
        const Eigen::Vector3f& position =
            point_cloud_->points[indices_[i]].getVector3fMap();
        // similarity of estimated plane normal to point normal
        normal_similarities[i] = std::abs(patch_->normal_.dot(normal));
        // distance from estimated plane to point
        point_distances[i] = std::abs(patch_->normal_.dot(position) +
            patch_->dist_from_origin_);
    }

    float tmp;
    // Use lower bound of the spread around the median as an indication
    // of how similar the point normals associated with the patch are.
    GetMinMaxRScore(normal_similarities, min_normal_diff_, tmp, 3);
    // Use upper bound of the spread around the median as an indication
    // of how close the points associated with the patch are to the patch.
    GetMinMaxRScore(point_distances, tmp, max_point_dist_, 3);

    // Fail if too much "variance" in how similar point normals are to patch
    // normal
    if (!IsNormalValid()) return false;

    // Fail if too much "variance" in distances of points to patch
    if (!IsDistanceValid()) return false;

    // Detect outliers, fail if too many
    std::unordered_map<size_t, bool> outliers;
    outliers.reserve(N);
    size_t num_outliers = 0;
    for (size_t i = 0; i < N; i++) {
        const bool is_outlier = normal_similarities[i] < min_normal_diff_ ||
            point_distances[i] > max_point_dist_;
        outliers[indices_[i]] = is_outlier;
        num_outliers += static_cast<int>(is_outlier);
    }
    if (num_outliers > N * outlier_ratio_thr_) return false;

    // Remove outliers
    if (num_outliers > 0) {
        indices_.erase(std::remove_if(indices_.begin(), indices_.end(),
            [&outliers](const size_t& idx) {
                return outliers[idx];
            }),
            indices_.end());
    }

    return true;
}

/// \brief Check if detected plane normal is similar to point normals.

bool PlaneDetector::IsNormalValid() const {
    return min_normal_diff_ > normal_similarity_thr_;
}

/// \brief Check if point distances from detected plane are reasonable.
///
/// Constructs an auxiliary vector which captures
/// coplanarity and curvature of points.

bool PlaneDetector::IsDistanceValid() /*const*/ {
    // Test point-to-plane distance w.r.t coplanarity of points.
    // See Fig. 4 of [ArujoOliveira2020].
    const Eigen::Vector3f F =
        (B_.col(0) * longest_edge_ + patch_->normal_ * max_point_dist_)
        .normalized();
    return std::abs(F.dot(patch_->normal_)) < coplanarity_thr_;
}

/// \brief Find perimeter of 3D points describing a plane
///
/// \param M  3D perimeter points
void PlaneDetector::GetPlanePerimeterPoints(Eigen::Matrix3Xf& M) {
    // project each point onto the 2D span (x-y) of orthogonal basis
    std::vector<Eigen::Vector2f> projectedPoints2d(indices_.size());
    for (size_t i = 0; i < indices_.size(); i++) {
        const Eigen::Vector3f& p = point_cloud_->points[indices_[i]].getArray3fMap();

        const float u = p.dot(B_.col(0));
        const float v = p.dot(B_.col(1));
        projectedPoints2d[i] << u, v;
    }

    std::vector<size_t> perimeter;
    GetConvexHull2D(projectedPoints2d, perimeter);

    M = Eigen::Matrix3Xf(3, perimeter.size());
    for (size_t i = 0; i < perimeter.size(); i++) {
        M.col(i) = point_cloud_->points[indices_[perimeter[i]]].getVector3fMap();
    }
}

pcl::Vertices PlaneDetector::GetPlanePerimeterVertices() {
    std::vector<Eigen::Vector2f> projectedPoints2d(indices_.size());
    for (size_t i = 0; i < indices_.size(); i++) {
        const Eigen::Vector3f& p = point_cloud_->points[indices_[i]].getArray3fMap();

        const float u = p.dot(B_.col(0));
        const float v = p.dot(B_.col(1));
        projectedPoints2d[i] << u, v;
    }

    std::vector<size_t> perimeter;
    GetConvexHull2D(projectedPoints2d, perimeter);
    pcl::Vertices vertices;

    for (size_t pidx : perimeter) {
        vertices.vertices.push_back(pidx);
    }

	return vertices;
}

PlaneDetector::RotatedRect::RotatedRect(const Eigen::Matrix3Xf& M, const Eigen::Matrix3f& B, float degrees) {
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(degrees * M_PI / 180.,
        Eigen::Vector3f::UnitZ());

    this->B = B * R;
    this->M = this->B.transpose() * M;
    bottom_left = this->M.rowwise().minCoeff();
    top_right = this->M.rowwise().maxCoeff();
    const float w = top_right(0) - bottom_left(0);
    const float h = top_right(1) - bottom_left(1);
    area = w * h;
}

/// \brief Builds an orthogonal basis of the plane using the normal for up.
///
/// The constructed basis matrix can be interpreted as the rotation
/// of the plane w.r.t the world (or, the point cloud sensor) frame.
/// In other words, each column of B = [x^w_p y^w_p z^w_p] is one of
/// the plane's basis vectors expressed in the world frame. Therefore,
/// the matrix B ( = R^w_p) can be used to express the plane points M
/// (expressed in the world frame) into the "plane" frame via M' = B^T*M
///
/// \param B The 3x3 basis matrix.
void PlaneDetector::ConstructOrthogonalBasis(Eigen::Matrix3f& B) {
    static constexpr float tol = 1e-3;
    if ((Eigen::Vector3f(0, 1, 1) - patch_->normal_).squaredNorm() > tol) {
        // construct x-vec by cross(normal, [0;1;1])
        B.col(0) =
            Eigen::Vector3f(patch_->normal_.y() - patch_->normal_.z(),
                -patch_->normal_.x(), patch_->normal_.x())
            .normalized();
    }
    else {
        // construct x-vec by cross(normal, [1;0;1])
        B.col(0) =
            Eigen::Vector3f(patch_->normal_.y(),
                patch_->normal_.z() - patch_->normal_.x(),
                -patch_->normal_.y())
            .normalized();
    }
    B.col(1) = patch_->normal_.cross(B.col(0)).normalized();
    B.col(2) = patch_->normal_;
}
