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
	auto b = voxels.size();
	for (auto& el : voxels) {
		voxel_vector.push_back(el.second);
	}
	return voxel_vector;
}

Eigen::Vector3i blast::Voxel_grid::get_voxel(const Eigen::Vector3d& point) const
{
	Eigen::Vector3d voxel_f = (point - origin) / voxel_size;
	return (Eigen::floor(voxel_f.array())).cast<int>();
}

std::vector<blast::Voxel> blast::Voxel_grid::get_voxels_around(const Eigen::Vector3i idx, int distance) const
{
	std::vector<Voxel> voxels_around;
	for (int x = -distance; x <= distance; ++x) {
		for (int y = -distance; y <= distance; ++y) {
			for (int z = -distance; z <= distance; ++z) {
				Eigen::Vector3i new_idx = idx + Eigen::Vector3i(x, y, z);
				if (has_voxel(new_idx)) {
					voxels_around.push_back(voxels.at(new_idx));
				}
			}
		}
	}

	return voxels_around;
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
		if (!output->has_voxel(idx)) {
			output->add_voxel(voxel);
		}
	}

	return output;
}

std::shared_ptr<blast::Voxel_grid> blast::dilate_grid(Voxel_grid& voxel_grid, int dilation_amount)
{
	auto voxels = voxel_grid.get_voxels();
	auto voxel_count = voxels.size();
	auto voxel_size = voxel_grid.voxel_size;
	auto origin = voxel_grid.origin;

	auto output = std::make_shared<Voxel_grid>();
	output->voxel_size = voxel_size;
	output->origin = origin;

	for (size_t i = 0; i < voxel_count; ++i)
	{
		auto voxel = voxels[i];
		auto idx = voxel.grid_index;
		for (int x = -dilation_amount; x <= dilation_amount; ++x)
		{
			for (int y = -dilation_amount; y <= dilation_amount; ++y)
			{
				for (int z = -dilation_amount; z <= dilation_amount; ++z)
				{
					Eigen::Vector3i new_idx = idx + Eigen::Vector3i(x, y, z);
					if (!output->has_voxel(new_idx))
					{
						Voxel new_voxel{ new_idx };
						output->add_voxel(new_voxel);
					}
				}
			}
		}
	}

	return output;
}

std::shared_ptr<blast::Voxel_grid> blast::subtract_grids(Voxel_grid& grid1, Voxel_grid& grid2)
{
	auto voxels1 = grid1.get_voxels();
	auto voxels2 = grid2.get_voxels();
	auto voxel_count1 = voxels1.size();
	auto voxel_count2 = voxels2.size();
	auto voxel_size = grid1.voxel_size;
	auto origin = grid1.origin;

	auto output = std::make_shared<Voxel_grid>();
	output->voxel_size = voxel_size;
	output->origin = origin;

	for (size_t i = 0; i < voxel_count1; ++i)
	{
		auto voxel = voxels1[i];
		auto idx = voxel.grid_index;
		if (!grid2.has_voxel(idx))
		{
			output->add_voxel(voxel);
		}
	}

	return output;
}

std::vector<blast::Via_point> blast::generate_via_point_candidates(const Voxel_grid& voxel_grid, size_t amount, double potential_field_max_distance)
{
	std::vector<Via_point> via_points;
	auto voxels = voxel_grid.get_voxels();
	auto voxel_count = voxels.size();
	auto voxel_size = voxel_grid.voxel_size;
	auto origin = voxel_grid.origin;

	for (int i = 0; i < amount; ++i)
	{
		auto idx = voxels[rand() % voxel_count].grid_index;
		Eigen::Vector3d point = voxel_grid.get_voxel_center_coordinate(idx);
		Eigen::Vector3d direction = Eigen::Vector3d::Random().normalized();

		// TODO: Actual direction generation

		Via_point via_point{ point, direction };
		via_points.push_back(via_point);
	}


	return via_points;
}

