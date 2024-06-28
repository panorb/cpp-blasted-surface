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

	const std::vector<std::vector<float>> edges = {
		{ 0.f,  3.f,  4.f, 0.f, 2.f },
		{  3.f, 0.f,  2.f,  5.f, 1.f },
		{  4.f,  2.f, 0.f,  1.f, 6.f },
		{ 0.f,  5.f,  1.f, 0.f, 3.f },
		{  2.f,  1.f,  6.f,  3.f, 0.f }
	};
	blast::add_edges_from_adjacency_matrix(graph, edges);

	blast::Ant_colony_optimizer aco{ 5,1 };
	std::vector<size_t> path = aco.execute(graph);

	std::unordered_set<size_t> included_nodes{ path.begin(), path.end() };

	REQUIRE(included_nodes.size() == path.size()); // No duplicates
	REQUIRE(included_nodes.size() == graph.get_node_count()); // All nodes are in the path
	for (size_t i = 0; i < path.size() - 1; i++)
	{
		REQUIRE(graph.exists_edge(path[i], path[i + 1])); // Path is valid
	}
}