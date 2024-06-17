#include <blast/ant_colony_optimizer.hpp>
#include <catch.hpp>

TEST_CASE("AntColonyOptimizer", "[aco]")
{
	blast::Graph graph;
	REQUIRE(graph.get_node_count() == 0);

	graph.add_node(std::make_shared<blast::Node>("Node A"));
	graph.add_node(std::make_shared<blast::Node>("Node B"));
	graph.add_node(std::make_shared<blast::Node>("Node C"));
	graph.add_node(std::make_shared<blast::Node>("Node D"));
	graph.add_node(std::make_shared<blast::Node>("Node E"));

	const std::vector<std::vector<int>> edges = {
		{ -1,  3,  4, -1, 2 },
		{  3, -1,  2,  5, 1 },
		{  4,  2, -1,  1, 6 },
		{ -1,  5,  1, -1, 3 },
		{  2,  1,  6,  3, -1 }
	};

	blast::add_edges_from_adjacency_matrix(graph, edges);
}