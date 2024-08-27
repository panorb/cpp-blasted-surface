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

/// \brief Calculate the median of a buffer of data
///
/// \param buffer Container of scalar data to find median of. A copy is made
/// so that buffer may be sorted.
/// \return Median of buffer data.
float GetMedian(std::vector<float> buffer);

/// \brief Calculate the Median Absolute Deviation statistic
///
/// \param buffer Container of scalar data to find MAD of.
/// \param median Precomputed median of buffer.
/// \return MAD = median(| X_i - median(X) |)
float GetMAD(const std::vector<float>& buffer, float median);

/// \brief Calculate spread of data as interval around median
///
/// I = [min, max] = [median(X) - α·MAD, median(X) + α·MAD]
///
/// \param buffer Container of scalar data to find spread of.
/// \param min Alpha MADs below median.
/// \param max Alpha MADs above median.
void GetMinMaxRScore(const std::vector<float>& buffer,
    float& min,
    float& max,
    float alpha);

/// \brief Identify the 2D convex hull of a set of 2D points.
///
/// \param points  Set of 2D points to find convex hull of.
/// \param indices  Indices of resulting convex hull.
void GetConvexHull2D(const std::vector<Eigen::Vector2f>& points,
    std::vector<size_t>& indices);
