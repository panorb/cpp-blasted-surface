#include <blast/ant_colony_optimizer.hpp>
#include <catch.hpp>

TEST_CASE("AntColonyOptimizer", "[aco]")
{
	blast::Graph graph;
	REQUIRE(graph.get_node_count() == 0);
}