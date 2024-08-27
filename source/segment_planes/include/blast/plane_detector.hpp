#pragma once

#include "segmenter_utility.hpp"
#include "bounding_box.hpp"
#include "planar_patch.hpp"

/// \class PlaneDetector
///
/// \brief Robust detection of planes from point sets
///
/// Using median as a robust estimator, PlaneDetector consumes a point cloud
/// and estimate a single plane using the point positions and their normals.
/// Planarity is verified by statistics-based tests on the associated points.
/// This implementation follows the work of [ArujoOliveira2020] as outlined in
///
///     Araújo and Oliveira, “A robust statistics approach for plane
///     detection in unorganized point clouds,” Pattern Recognition, 2020.
///
/// See also https://www.inf.ufrgs.br/~oliveira/pubs_files/RE/RE.html
class PlaneDetector {
public:
    /// \brief Constructor to initialize detection parameters.
    ///
    /// \param normal_similarity is the min allowable similarity score
    /// between a point normal and the detected plane normal.
    /// \param coplanarity is the max allowable similiarity between
    /// detected plane normal and auxiliary planarity test vector. An
    /// ideal plane has score 0, i.e., normal orthogonal to test vector.
    /// \param outlier_ratio is the max allowable ratio of outlier points.
    PlaneDetector(float normal_similarity,
        float coplanarity,
        float outlier_ratio,
        float plane_edge_length)
        : patch_(std::make_shared<PlanarPatch>()),
        normal_similarity_thr_(normal_similarity),
        coplanarity_thr_(coplanarity),
        outlier_ratio_thr_(outlier_ratio),
        plane_edge_length_thr_(plane_edge_length) {}
    ~PlaneDetector() = default;

    /// \brief Estimate plane from point cloud and selected point indices.
    ///
    /// \param point_cloud The point cloud (with points and normals).
    /// \param indices Point indices within point cloud to use.
    /// \return True if indices of point cloud pass robust planarity tests.
    bool DetectFromPointCloud(const SPointCloud* point_cloud, const SNormalCloud* normal_cloud,
        const std::vector<size_t>& indices);

    /// \brief Delimit the plane using its perimeter points.
    ///
    /// \return A patch which is the bounded version of the plane.
    std::shared_ptr<OrientedBoundingBox> DelimitPlane();

    /// \brief Determine if a point is an inlier to the estimated plane model.
    ///
    /// \param idx  Index of point in point_cloud_
    bool IsInlier(size_t idx);

    /// \brief Check if cloud point at index idx has been visited.
    bool HasVisited(size_t idx);

    /// \brief Mark the cloud point at index idx as visited.
    void MarkVisited(size_t idx);

    /// \brief Include an addition point at index idx as part of plane set.
    void AddPoint(size_t idx);

    /// \brief Estimate the plane parameters again to include added points.
    void Update();

    /// \brief Check if plane is considered a false positive
    bool IsFalsePositive();

public:
    /// Patch object which is a bounded version of the plane
    std::shared_ptr<PlanarPatch> patch_;
    /// Underlying point cloud object containing all points
    const SPointCloud* point_cloud_;
    /// Underlying normal cloud object containing all normals
    const SNormalCloud* normal_cloud_;
    /// Associated indices of points in the underlying point cloud
    std::vector<size_t> indices_;

    /// Minimum tail of the spread in normal similarity scores
    float min_normal_diff_;
    /// Maximum tail of the spread of point distances from plane
    float max_point_dist_;

    /// Indicates that the plane cannot grow anymore
    bool stable_ = false;

    /// Number of new points from grow and/or merge stage
    size_t num_new_points_ = 0;
    /// Number of times this plane has needed to re-estimate plane parameters
    size_t num_updates_ = 0;

    /// A given index of this plane for merging purposes
    size_t index_;

    pcl::Vertices GetPlanePerimeterVertices();

private:
    /// Bounds of the estimate plane
    Eigen::Vector3f min_bound_;
    Eigen::Vector3f max_bound_;
    float longest_edge_;

    /// Orthogonal basis of the estimated plane
    Eigen::Matrix3f B_;

    /// Minimum allowable similarity score for point normal to plane normal.
    float normal_similarity_thr_;
    /// Maximum allowable similarity between normal and auxiliary planarity test
    /// vector. An ideal plane has score 0, i.e., normal orthogonal to test
    /// vector.
    float coplanarity_thr_;
    /// Maximum allowable outlier ratio
    float outlier_ratio_thr_;
    /// The longest edge of resulting patch must be larger than this.
    float plane_edge_length_thr_;

    /// Visited list of points during grow and/or merge stages.
    std::unordered_set<size_t> visited_indices_;

    /// \brief Rotates an orthogonal basis, creating a new rotated basis
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

    /// \brief Estimate plane from point cloud and selected point indices.
    void EstimatePlane();

    /// \brief Use robust statistics (i.e., median) to test planarity.
    ///
    /// Follows Sec 3.2 of [ArujoOliveira2020].
    ///
    /// \return True if passes tests.
    bool RobustPlanarityTest();

    /// \brief Check if detected plane normal is similar to point normals.
    inline bool IsNormalValid() const;

    /// \brief Check if point distances from detected plane are reasonable.
    ///
    /// Constructs an auxiliary vector which captures
    /// coplanarity and curvature of points.
    bool IsDistanceValid();

    /// \brief Find perimeter of 3D points describing a plane
    ///
    /// \param M  3D perimeter points
    void GetPlanePerimeterPoints(Eigen::Matrix3Xf& M);

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
    void ConstructOrthogonalBasis(Eigen::Matrix3f& B);
};

using PlaneDetectorPtr = std::shared_ptr<PlaneDetector>;
