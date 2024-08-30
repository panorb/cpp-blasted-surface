#include "blast/planes/segmenter_utility.hpp"

float get_median(std::vector<float> buffer) {
    const size_t N = buffer.size();
    std::nth_element(buffer.begin(), buffer.begin() + N / 2,
        buffer.begin() + N);
    return buffer[N / 2];
}

/// \return MAD = median(| X_i - median(X) |)
float get_mad(const std::vector<float>& buffer, float median) {
    const size_t N = buffer.size();
    std::vector<float> shifted(N);
    for (size_t i = 0; i < N; i++) {
        shifted[i] = std::abs(buffer[i] - median);
    }
    std::nth_element(shifted.begin(), shifted.begin() + N / 2,
        shifted.begin() + N);
    static constexpr float k = 1.4826;  // assumes normally distributed data
    return k * shifted[N / 2];
}

void get_min_max_r_score(const std::vector<float>& buffer, float& min, float& max, float alpha) {
    float median = get_median(buffer);
    float mad = get_mad(buffer, median);
    min = median - alpha * mad;
    max = median + alpha * mad;
}

/// \param points  Set of 2D points to find convex hull of.
/// \param indices  Indices of resulting convex hull.
void get_convex_hull_2d(const std::vector<Eigen::Vector2f>& points, std::vector<size_t>& indices) {
    static constexpr int DIM = 2;
    std::vector<size_t> pt_map;

    std::vector<double> qhull_points_data(points.size() * DIM);
    for (size_t pidx = 0; pidx < points.size(); ++pidx) {
        const auto& pt = points[pidx].cast<double>();
        qhull_points_data[pidx * DIM + 0] = (double) pt(0);
        qhull_points_data[pidx * DIM + 1] = (double) pt(1);
    }

    orgQhull::PointCoordinates qhull_points(DIM, "");
    qhull_points.append(qhull_points_data);

    orgQhull::Qhull qhull;
    qhull.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
        qhull_points.count(), qhull_points.coordinates(), "");

    orgQhull::QhullVertexList vertices = qhull.vertexList();
    indices.clear();
    indices.reserve(vertices.size());
    for (orgQhull::QhullVertexList::iterator it = vertices.begin();
        it != vertices.end(); ++it) {
        indices.push_back(it->point().id());
    }
}
