#pragma once

#include <unordered_map>
#include <Eigen/Core>

#include <blast/utility.hpp>

class Voxel
{
public:
	Voxel() = default;
	Voxel(Eigen::Vector3i& grid_index) : grid_index(grid_index) {};

	Eigen::Vector3i grid_index;
};

class Voxel_grid {
public:
	Voxel_grid(double voxel_size) : voxel_size(voxel_size) {};
	void add_voxel(const Voxel& voxel);
	void remove_voxel(const Eigen::Vector3i& idx);
	bool has_voxels() const {
		return !voxels.empty();
	}
	std::vector<Voxel> get_voxels() const;
	Eigen::Vector3i get_voxel(const Eigen::Vector3d& point) const;
	Eigen::Vector3d get_voxel_center_coordinate(const Eigen::Vector3i& idx) const {
		auto it = voxels.find(idx);
		if (it != voxels.end()) {
			auto voxel = it->second;
			return ((voxel.grid_index.cast<double>() +
				Eigen::Vector3d(0.5, 0.5, 0.5)) *
				voxel_size) +
				origin;
		}

		return Eigen::Vector3d::Zero();
	}

	std::vector<Eigen::Vector3d> get_voxel_bounding_points(const Eigen::Vector3i& idx) const;
private:
	double voxel_size;
	Eigen::Vector3d origin = Eigen::Vector3d::Zero();
	//std::vector<std::vector<bool>> active_voxels;
	std::unordered_map<Eigen::Vector3i, Voxel, utility::hash_eigen<Eigen::Vector3i>> voxels;
};
