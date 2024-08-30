#pragma once

#include "segmenter_utility.hpp"

class Oriented_bounding_box {
public:
                Oriented_bounding_box()
        : center_(0, 0, 0),
        R_(Eigen::Matrix3f::Identity()),
        extent_(0, 0, 0),
        color_(1, 1, 1) {}
                            Oriented_bounding_box(const Eigen::Vector3f& center,
        const Eigen::Matrix3f& R,
        const Eigen::Vector3f& extent)
        : center_(center),
        R_(R),
        extent_(extent) {}
    ~Oriented_bounding_box() {}

public:
    Oriented_bounding_box& clear();
    bool is_empty() const;
    virtual Eigen::Vector3f get_center() const;

        virtual Oriented_bounding_box get_oriented_bounding_box(
        bool robust) const;

        virtual Oriented_bounding_box get_minimal_oriented_bounding_box(
        bool robust) const;

    virtual Oriented_bounding_box& transform(
        const Eigen::Matrix4f& transformation);
    virtual Oriented_bounding_box& translate(const Eigen::Vector3f& translation,
        bool relative = true);
    virtual Oriented_bounding_box& scale(const double scale,
        const Eigen::Vector3f& center);
    virtual Oriented_bounding_box& rotate(const Eigen::Matrix3f& R,
        const Eigen::Vector3f& center);

        double volume() const;

            ///      ------- x
    ///     /|
    ///    / |
    ///   /  | z
    ///  y
    ///      0 ------------------- 1
    ///       /|                /|
    ///      / |               / |
    ///     /  |              /  |
    ///    /   |             /   |
    /// 2 ------------------- 7  |
    ///   |    |____________|____| 6
    ///   |   /3            |   /
    ///   |  /              |  /
    ///   | /               | /
    ///   |/                |/
            std::vector<Eigen::Vector3f> get_box_points() const;

        std::vector<size_t> get_point_indices_within_bounding_box(
        const std::vector<Eigen::Vector3f>& points) const;

                                ///               in degenerate cases but introduces noise to the points
    ///               coordinates.
    static Oriented_bounding_box create_from_points(
        const std::vector<Eigen::Vector3f>& points, bool robust = false);

                                    ///               in degenerate cases but introduces noise to the points
    ///               coordinates.
    static Oriented_bounding_box create_from_points_minimal(
        const std::vector<Eigen::Vector3f>& points, bool robust = false);

public:
        Eigen::Vector3f center_;
            Eigen::Matrix3f R_;
        Eigen::Vector3f extent_;
        Eigen::Vector3f color_;
};

