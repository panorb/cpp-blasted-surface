#pragma once

#define NOMINMAX
#define _USE_MATH_DEFINES

#include <spdlog/spdlog.h>

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <math.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/PointCoordinates.h>
#include <chrono>
#include <numeric>
#include <unordered_set>
#include <pcl/Vertices.h>

using SPoint = pcl::PointXYZ;
using SNormal = pcl::Normal;
using SPointCloud = pcl::PointCloud<SPoint>;
using SNormalCloud = pcl::PointCloud<SNormal>;

// using namespace pcl;
using namespace std::chrono;

float get_median(std::vector<float> buffer);

/// \return MAD = median(| X_i - median(X) |)
float get_mad(const std::vector<float>& buffer, float median);

void get_min_max_r_score(const std::vector<float>& buffer,
    float& min,
    float& max,
    float alpha);

/// \param points  Set of 2D points to find convex hull of.
/// \param indices  Indices of resulting convex hull.
void get_convex_hull_2d(const std::vector<Eigen::Vector2f>& points,
    std::vector<size_t>& indices);
