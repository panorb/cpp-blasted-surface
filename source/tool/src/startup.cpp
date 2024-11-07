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
	tool.selected_example_file = tbl["input"]["pointcloud_file"].value_or("");

	tool.plane_segmenter.normal_similarity_deg_ = tbl["oliveira"]["normal_similarity_deg"].value_or(60.0);
	tool.plane_segmenter.coplanarity_deg_ = tbl["oliveira"]["coplanarity_deg"].value_or(75.0);
	tool.plane_segmenter.outlier_ratio_ = tbl["oliveira"]["outlier_ratio"].value_or(0.75);
	tool.plane_segmenter.min_plane_edge_length_ = tbl["oliveira"]["min_plane_edge_length"].value_or(0.0);
	tool.plane_segmenter.min_num_points_ = tbl["oliveira"]["min_num_points"].value_or(0);



	tool.show();

	// Write back to config...
	// tbl["input"]["pointcloud_file"].ref<std::string> = ;
	// tbl.at("input.pointcloud_file").ref<std::string>() = tool.selected_example_file;

	tbl["input"].as_table()->insert_or_assign("pointcloud_file", tool.selected_example_file);
	tbl["oliveira"].as_table()->insert_or_assign("normal_similarity_deg", tool.plane_segmenter.normal_similarity_deg_);
	
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
