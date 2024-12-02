#define _USE_MATH_DEFINES
#include <spdlog/spdlog.h>

#include "blast/tool.hpp"
#include <toml++/toml.hpp>

int main(int argc, char** argv)
{
	toml::table tbl;
	try
	{
		tbl = toml::parse_file("config.toml");
	} catch (const toml::parse_error& err)
	{
		spdlog::error("Error parsing file '{}':\n{}\n({}, {})", err.source().path->c_str(), err.description(), err.source().begin.line, err.source().begin.column);
		return 1;
	}

	blast::Tool tool;
	// Read from config...
	tool.selected_example_file = tbl["input"]["pointcloud_file"].value_or(tool.selected_example_file);

	tool.plane_segmenter.normal_similarity_deg_ = tbl["oliveira"]["normal_similarity_deg"].value_or(tool.plane_segmenter.normal_similarity_deg_);
	tool.plane_segmenter.coplanarity_deg_ = tbl["oliveira"]["coplanarity_deg"].value_or(tool.plane_segmenter.coplanarity_deg_);
	tool.plane_segmenter.outlier_ratio_ = tbl["oliveira"]["outlier_ratio"].value_or(tool.plane_segmenter.outlier_ratio_);
	tool.plane_segmenter.min_plane_edge_length_ = tbl["oliveira"]["min_plane_edge_length"].value_or(tool.plane_segmenter.min_plane_edge_length_);
	tool.plane_segmenter.min_num_points_ = tbl["oliveira"]["min_num_points"].value_or(tool.plane_segmenter.min_num_points_);

	tool.grid_sampler.grid_size_ = tbl["grid_sampler"]["grid_size"].value_or(tool.grid_sampler.grid_size_);
	tool.grid_sampler.circle_radius_ = tbl["grid_sampler"]["circle_radius"].value_or(tool.grid_sampler.circle_radius_);
	tool.grid_sampler.erosion_iterations_ = tbl["grid_sampler"]["erosion_iterations"].value_or(tool.grid_sampler.erosion_iterations_);
	tool.grid_sampler.hole_filling_enabled_ = tbl["grid_sampler"]["hole_filling_enabled"].value_or(tool.grid_sampler.hole_filling_enabled_);

	tool.show();

	// Write back to config...
	// tbl["input"]["pointcloud_file"].ref<std::string> = ;
	// tbl.at("input.pointcloud_file").ref<std::string>() = tool.selected_example_file;

	tbl["input"].as_table()->insert_or_assign("pointcloud_file", tool.selected_example_file);

	auto oliveira_tbl = tbl["oliveira"].as_table();
	oliveira_tbl->insert_or_assign("normal_similarity_deg", tool.plane_segmenter.normal_similarity_deg_);
	oliveira_tbl->insert_or_assign("coplanarity_deg", tool.plane_segmenter.coplanarity_deg_);
	oliveira_tbl->insert_or_assign("outlier_ratio", tool.plane_segmenter.outlier_ratio_);
	oliveira_tbl->insert_or_assign("min_plane_edge_length", tool.plane_segmenter.min_plane_edge_length_);
	oliveira_tbl->insert_or_assign("min_num_points", tool.plane_segmenter.min_num_points_);

	auto grid_sampler_tbl = tbl["grid_sampler"].as_table();
	grid_sampler_tbl->insert_or_assign("grid_size", tool.grid_sampler.grid_size_);
	grid_sampler_tbl->insert_or_assign("circle_radius", tool.grid_sampler.circle_radius_);
	grid_sampler_tbl->insert_or_assign("erosion_iterations", tool.grid_sampler.erosion_iterations_);
	grid_sampler_tbl->insert_or_assign("hole_filling_enabled", tool.grid_sampler.hole_filling_enabled_);

	//*tbl["oliveira"]["coplanarity_deg"].as_floating_point() = tool.plane_segmenter.coplanarity_deg_;
	//*tbl["oliveira"]["outlier_ratio"].as_floating_point() = tool.plane_segmenter.outlier_ratio_;
	//*tbl["oliveira"]["min_plane_edge_length"].as_floating_point() = tool.plane_segmenter.min_plane_edge_length_;
	//*tbl["oliveira"]["min_num_points"].as_integer() = tool.plane_segmenter.min_num_points_;

	std::ofstream config_stream;
	config_stream.open("config.toml");

	config_stream << tbl;

	// tool.run();
	return 0;
}
