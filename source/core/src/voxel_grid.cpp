#include "blast/voxel_grid.hpp"

#include <Eigen/src/Core/GlobalFunctions.h>

void blast::Voxel_grid::add_voxel(const Voxel& voxel)
{
	voxels[voxel.grid_index] = voxel;
}

void blast::Voxel_grid::remove_voxel(const Eigen::Vector3i& idx)
{
	voxels.erase(idx);
}

std::vector<blast::Voxel> blast::Voxel_grid::get_voxels() const
{
	std::vector<Voxel> voxel_vector;
	for (const auto& [key, value] : voxels) {
		voxel_vector.push_back(value);
	}
	return voxel_vector;
}

Eigen::Vector3i blast::Voxel_grid::get_voxel(const Eigen::Vector3d& point) const
{
	Eigen::Vector3d voxel_f = (point - origin) / voxel_size;
	return (Eigen::floor(voxel_f.array())).cast<int>();
}

std::vector<Eigen::Vector3d> blast::Voxel_grid::get_voxel_bounding_points(const Eigen::Vector3i& idx) const
{
	double r = voxel_size / 2.0;
	auto x = get_voxel_center_coordinate(idx);
	std::vector<Eigen::Vector3d> points;
	points.push_back(x + Eigen::Vector3d(-r, -r, -r));
	points.push_back(x + Eigen::Vector3d(-r, -r, r));
	points.push_back(x + Eigen::Vector3d(r, -r, -r));
	points.push_back(x + Eigen::Vector3d(r, -r, r));
	points.push_back(x + Eigen::Vector3d(-r, r, -r));
	points.push_back(x + Eigen::Vector3d(-r, r, r));
	points.push_back(x + Eigen::Vector3d(r, r, -r));
	points.push_back(x + Eigen::Vector3d(r, r, r));
	return points;
}

std::shared_ptr<blast::Voxel_grid> blast::Voxel_grid::create_from_point_cloud(const Point_cloud& input, double voxel_size)
{
	Eigen::Vector3d voxel_size3{ voxel_size, voxel_size, voxel_size };
	Eigen::Vector3d min_bound = input.compute_min_bound() - voxel_size3 * 0.5;
	Eigen::Vector3d max_bound = input.compute_max_bound() + voxel_size3 * 0.5;
	return create_from_point_cloud_within_bounds(input, voxel_size, min_bound, max_bound);
}

std::shared_ptr<blast::Voxel_grid> blast::Voxel_grid::create_from_point_cloud_within_bounds(const Point_cloud& input,
	double voxel_size, const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound)
{
	auto output = std::make_shared<Voxel_grid>();
	if (voxel_size <= 0.0) {
		throw std::invalid_argument("Voxel size must be greater than 0");
	}

	output->voxel_size = voxel_size;
	output->origin = min_bound;

	for (const auto& point : input.get_points()) {
		if (point.x() < min_bound.x() || point.y() < min_bound.y() || point.z() < min_bound.z() ||
			point.x() > max_bound.x() || point.y() > max_bound.y() || point.z() > max_bound.z()) {
			continue;
		}

		Eigen::Vector3i idx = output->get_voxel(point);
		Voxel voxel{ idx };
		output->add_voxel(voxel);
	}
}

