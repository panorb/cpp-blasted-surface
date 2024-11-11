#include "blast/planes/bounding_box.hpp"


Oriented_bounding_box& Oriented_bounding_box::clear() {
    center_.setZero();
    extent_.setZero();
    R_ = Eigen::Matrix3f::Identity();
    color_.setOnes();
    return *this;
}

bool Oriented_bounding_box::is_empty() const { return volume() <= 0; }

Eigen::Vector3f Oriented_bounding_box::get_center() const { return center_; }

Oriented_bounding_box Oriented_bounding_box::get_oriented_bounding_box(bool) const {
    return *this;
}

Oriented_bounding_box Oriented_bounding_box::get_minimal_oriented_bounding_box(
    bool) const {
    return *this;
}

Oriented_bounding_box& Oriented_bounding_box::transform(
    const Eigen::Matrix4f& transformation) {
    spdlog::error(
        "A general transform of an Oriented_bounding_box is not implemented. "
        "Call translate, scale, and rotate.");
    return *this;
}

Oriented_bounding_box& Oriented_bounding_box::translate(
    const Eigen::Vector3f& translation, bool relative) {
    if (relative) {
        center_ += translation;
    }
    else {
        center_ = translation;
    }
    return *this;
}

Oriented_bounding_box& Oriented_bounding_box::scale(const double scale,
    const Eigen::Vector3f& center) {
    extent_ *= scale;
    center_ = scale * (center_ - center) + center;
    return *this;
}

Oriented_bounding_box& Oriented_bounding_box::rotate(
    const Eigen::Matrix3f& R, const Eigen::Vector3f& center) {
    R_ = R * R_;
    center_ = R * (center_ - center) + center;
    return *this;
}

double Oriented_bounding_box::area() const{
    return extent_(0) * extent_(1);
}

double Oriented_bounding_box::volume() const {
    return extent_(0) * extent_(1) * extent_(2);
}

std::vector<Eigen::Vector3f> Oriented_bounding_box::get_box_points() const {
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

bool Oriented_bounding_box::collides_with(Oriented_bounding_box& other_box)
{
	Eigen::Vector3f dx = R_ * Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f dy = R_ * Eigen::Vector3f(0, 1, 0);
	Eigen::Vector3f dz = R_ * Eigen::Vector3f(0, 0, 1);
	Eigen::Vector3f other_dx = other_box.R_ * Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f other_dy = other_box.R_ * Eigen::Vector3f(0, 1, 0);
	Eigen::Vector3f other_dz = other_box.R_ * Eigen::Vector3f(0, 0, 1);
	Eigen::Vector3f d = other_box.center_ - center_;
	return std::abs(d.dot(dx)) <= extent_(0) / 2 + other_box.extent_(0) / 2 &&
		std::abs(d.dot(dy)) <= extent_(1) / 2 + other_box.extent_(1) / 2 &&
		std::abs(d.dot(dz)) <= extent_(2) / 2 + other_box.extent_(2) / 2 &&
		std::abs(d.dot(other_dx)) <= extent_(0) / 2 + other_box.extent_(0) / 2 &&
		std::abs(d.dot(other_dy)) <= extent_(1) / 2 + other_box.extent_(1) / 2 &&
		std::abs(d.dot(other_dz)) <= extent_(2) / 2 + other_box.extent_(2) / 2 &&
		std::abs(d.dot(dx.cross(other_dx))) <= extent_(1) / 2 * other_box.extent_(2) / 2 + extent_(2) / 2 * other_box.extent_(1) / 2 &&
		std::abs(d.dot(dy.cross(other_dy))) <= extent_(0) / 2 * other_box.extent_(2) / 2 + extent_(2) / 2 * other_box.extent_(0) / 2 &&
		std::abs(d.dot(dz.cross(other_dz))) <= extent_(0) / 2 * other_box.extent_(1) / 2 + extent_(1) / 2 * other_box.extent_(0) / 2;
}

std::vector<size_t> Oriented_bounding_box::get_point_indices_within_bounding_box(
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
