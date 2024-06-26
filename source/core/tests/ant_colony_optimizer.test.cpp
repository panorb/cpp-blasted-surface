#include <blast/ant_colony_optimizer.hpp>
#include <catch.hpp>
#include <unordered_set>

TEST_CASE("The Ant colony optimizer works as expected", "[aco]")
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

	blast::Ant_colony_optimizer aco{ 5,1 };
	std::vector<size_t> path = aco.execute(graph);

	std::unordered_set<size_t> included_nodes{ path.begin(), path.end() };
	
	REQUIRE(included_nodes.size() == path.size()); // No duplicates
	for (size_t i = 0; i < path.size() - 1; i++)
	{
		REQUIRE(graph.exists_edge(path[i], path[i + 1])); // Path is valid
	}
}