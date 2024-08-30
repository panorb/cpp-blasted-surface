#pragma once

#include <unordered_map>
#include <Eigen/Core>

#include <blast/utility.hpp>

#include "point_cloud.hpp"

namespace blast {
	struct Via_point
	{
		Via_point() = default;
		Via_point(Eigen::Vector3d& point, Eigen::Vector3d& direction) : point(point), direction(direction) {};

		Eigen::Vector3d point;
		Eigen::Vector3d direction;
	};

class Voxel
{
public:
	Voxel() = default;
	Voxel(Eigen::Vector3i& grid_index) : grid_index(grid_index) {};

	Eigen::Vector3i grid_index;
};

class Voxel_grid {
public:
	Voxel_grid() = default;
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
	std::vector<Voxel> get_voxels_around(Eigen::Vector3i idx, int distance) const;

	bool has_voxel(const Eigen::Vector3i& idx) const
	{
		return voxels.find(idx) != voxels.end();
	}

	std::vector<Eigen::Vector3d> get_voxel_bounding_points(const Eigen::Vector3i& idx) const;

	static std::shared_ptr<Voxel_grid> create_from_point_cloud(const Point_cloud& input, double voxel_size);
	static std::shared_ptr<Voxel_grid> create_from_point_cloud_within_bounds(const Point_cloud& input, double voxel_size, const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound);
	double voxel_size;
	Eigen::Vector3d origin = Eigen::Vector3d::Zero();
private:
	//std::vector<std::vector<bool>> active_voxels;
	std::unordered_map<Eigen::Vector3i, Voxel, utility::hash_eigen<Eigen::Vector3i>> voxels;
};


std::shared_ptr<Voxel_grid> dilate_grid(Voxel_grid& voxel_grid, int dilation_amount);
std::shared_ptr<Voxel_grid> subtract_grids(Voxel_grid& grid1, Voxel_grid& grid2);
std::vector<Via_point> generate_via_point_candidates(const Voxel_grid& voxel_grid, size_t amount,
                                                    double potential_field_max_distance);
//std::pair<std::vector<>>

}
