#include "blast/voxel_grid.hpp"

#include <Eigen/src/Core/GlobalFunctions.h>

void Voxel_grid::add_voxel(const Voxel& voxel)
{
	voxels[voxel.grid_index] = voxel;
}

void Voxel_grid::remove_voxel(const Eigen::Vector3i& idx)
{
	voxels.erase(idx);
}

std::vector<Voxel> Voxel_grid::get_voxels() const
{
	std::vector<Voxel> voxel_vector;
	for (const auto& [key, value] : voxels) {
		voxel_vector.push_back(value);
	}
	return voxel_vector;
}

Eigen::Vector3i Voxel_grid::get_voxel(const Eigen::Vector3d& point) const
{
	Eigen::Vector3d voxel_f = (point - origin) / voxel_size;
	return (Eigen::floor(voxel_f.array())).cast<int>();
}

std::vector<Eigen::Vector3d> Voxel_grid::get_voxel_bounding_points(const Eigen::Vector3i& idx) const
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

