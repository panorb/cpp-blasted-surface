#pragma once

#include "segmenter_utility.hpp"

/// \class OrientedBoundingBox
///
/// \brief A bounding box oriented along an arbitrary frame of reference.
///
/// The oriented bounding box is defined by its center position, rotation
/// matrix and extent.
class OrientedBoundingBox {
public:
    /// \brief Default constructor.
    ///
    /// Creates an empty Oriented Bounding Box.
    OrientedBoundingBox()
        : center_(0, 0, 0),
        R_(Eigen::Matrix3f::Identity()),
        extent_(0, 0, 0),
        color_(1, 1, 1) {}
    /// \brief Parameterized constructor.
    ///
    /// \param center Specifies the center position of the bounding box.
    /// \param R The rotation matrix specifying the orientation of the
    /// bounding box with the original frame of reference.
    /// \param extent The extent of the bounding box.
    OrientedBoundingBox(const Eigen::Vector3f& center,
        const Eigen::Matrix3f& R,
        const Eigen::Vector3f& extent)
        : center_(center),
        R_(R),
        extent_(extent) {}
    ~OrientedBoundingBox() {}

public:
    OrientedBoundingBox& Clear();
    bool IsEmpty() const;
    virtual Eigen::Vector3f GetCenter() const;

    /// Returns the object itself
    virtual OrientedBoundingBox GetOrientedBoundingBox(
        bool robust) const;

    /// Returns the object itself
    virtual OrientedBoundingBox GetMinimalOrientedBoundingBox(
        bool robust) const;

    virtual OrientedBoundingBox& Transform(
        const Eigen::Matrix4f& transformation);
    virtual OrientedBoundingBox& Translate(const Eigen::Vector3f& translation,
        bool relative = true);
    virtual OrientedBoundingBox& Scale(const double scale,
        const Eigen::Vector3f& center);
    virtual OrientedBoundingBox& Rotate(const Eigen::Matrix3f& R,
        const Eigen::Vector3f& center);

    /// Returns the volume of the bounding box.
    double Volume() const;

    /// Returns the eight points that define the bounding box.
    /// \verbatim
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
    /// 5 ------------------- 4
    /// \endverbatim
    std::vector<Eigen::Vector3f> GetBoxPoints() const;

    /// Return indices to points that are within the bounding box.
    std::vector<size_t> GetPointIndicesWithinBoundingBox(
        const std::vector<Eigen::Vector3f>& points) const;

    /// Creates an oriented bounding box using a PCA.
    /// Note, that this is only an approximation to the minimum oriented
    /// bounding box that could be computed for example with O'Rourke's
    /// algorithm (cf. http://cs.smith.edu/~jorourke/Papers/MinVolBox.pdf,
    /// https://www.geometrictools.com/Documentation/MinimumVolumeBox.pdf)
    /// \param points The input points
    /// \param robust If set to true uses a more robust method which works
    ///               in degenerate cases but introduces noise to the points
    ///               coordinates.
    static OrientedBoundingBox CreateFromPoints(
        const std::vector<Eigen::Vector3f>& points, bool robust = false);

    /// Creates the oriented bounding box with the smallest volume.
    /// The algorithm makes use of the fact that at least one edge of
    /// the convex hull must be collinear with an edge of the minimum
    /// bounding box: for each triangle in the convex hull, calculate
    /// the minimal axis aligned box in the frame of that triangle.
    /// at the end, return the box with the smallest volume
    /// \param points The input points
    /// \param robust If set to true uses a more robust method which works
    ///               in degenerate cases but introduces noise to the points
    ///               coordinates.
    static OrientedBoundingBox CreateFromPointsMinimal(
        const std::vector<Eigen::Vector3f>& points, bool robust = false);

public:
    /// The center point of the bounding box.
    Eigen::Vector3f center_;
    /// The rotation matrix of the bounding box to transform the original frame
    /// of reference to the frame of this box.
    Eigen::Matrix3f R_;
    /// The extent of the bounding box in its frame of reference.
    Eigen::Vector3f extent_;
    /// The color of the bounding box in RGB.
    Eigen::Vector3f color_;
};

