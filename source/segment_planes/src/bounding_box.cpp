#include "blast/bounding_box.hpp"


OrientedBoundingBox& OrientedBoundingBox::Clear() {
    center_.setZero();
    extent_.setZero();
    R_ = Eigen::Matrix3f::Identity();
    color_.setOnes();
    return *this;
}

bool OrientedBoundingBox::IsEmpty() const { return Volume() <= 0; }

Eigen::Vector3f OrientedBoundingBox::GetCenter() const { return center_; }

OrientedBoundingBox OrientedBoundingBox::GetOrientedBoundingBox(bool) const {
    return *this;
}

OrientedBoundingBox OrientedBoundingBox::GetMinimalOrientedBoundingBox(
    bool) const {
    return *this;
}

OrientedBoundingBox& OrientedBoundingBox::Transform(
    const Eigen::Matrix4f& transformation) {
    spdlog::error(
        "A general transform of an OrientedBoundingBox is not implemented. "
        "Call Translate, Scale, and Rotate.");
    return *this;
}

OrientedBoundingBox& OrientedBoundingBox::Translate(
    const Eigen::Vector3f& translation, bool relative) {
    if (relative) {
        center_ += translation;
    }
    else {
        center_ = translation;
    }
    return *this;
}

OrientedBoundingBox& OrientedBoundingBox::Scale(const double scale,
    const Eigen::Vector3f& center) {
    extent_ *= scale;
    center_ = scale * (center_ - center) + center;
    return *this;
}

OrientedBoundingBox& OrientedBoundingBox::Rotate(
    const Eigen::Matrix3f& R, const Eigen::Vector3f& center) {
    R_ = R * R_;
    center_ = R * (center_ - center) + center;
    return *this;
}

double OrientedBoundingBox::Volume() const {
    return extent_(0) * extent_(1) * extent_(2);
}

std::vector<Eigen::Vector3f> OrientedBoundingBox::GetBoxPoints() const {
    Eigen::Vector3f x_axis = R_ * Eigen::Vector3f(extent_(0) / 2, 0, 0);
    Eigen::Vector3f y_axis = R_ * Eigen::Vector3f(0, extent_(1) / 2, 0);
    Eigen::Vector3f z_axis = R_ * Eigen::Vector3f(0, 0, extent_(2) / 2);
    std::vector<Eigen::Vector3f> points(8);
    points[0] = center_ - x_axis - y_axis - z_axis;
    points[1] = center_ + x_axis - y_axis - z_axis;
    points[2] = center_ - x_axis + y_axis - z_axis;
    points[3] = center_ - x_axis - y_axis + z_axis;
    points[4] = center_ + x_axis + y_axis + z_axis;
    points[5] = center_ - x_axis + y_axis + z_axis;
    points[6] = center_ + x_axis - y_axis + z_axis;
    points[7] = center_ + x_axis + y_axis - z_axis;
    return points;
}

std::vector<size_t> OrientedBoundingBox::GetPointIndicesWithinBoundingBox(
    const std::vector<Eigen::Vector3f>& points) const {
    std::vector<size_t> indices;
    Eigen::Vector3f dx = R_ * Eigen::Vector3f(1, 0, 0);
    Eigen::Vector3f dy = R_ * Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f dz = R_ * Eigen::Vector3f(0, 0, 1);
    for (size_t idx = 0; idx < points.size(); idx++) {
        Eigen::Vector3f d = points[idx] - center_;
        if (std::abs(d.dot(dx)) <= extent_(0) / 2 &&
            std::abs(d.dot(dy)) <= extent_(1) / 2 &&
            std::abs(d.dot(dz)) <= extent_(2) / 2) {
            indices.push_back(idx);
        }
    }
    return indices;
}
