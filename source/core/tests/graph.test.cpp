#include <graph.h>
#include <catch.hpp>

TEST_CASE("Create and add nodes to graph", "[graph]") {
	core::Graph graph;
	core::Node node{ "Test" };
	graph.addNode(node);

	REQUIRE(graph.getNode(0)->getLabel() == "Test");
}

TEST_CASE("Create and check for weighted directed edge", "[graph]") {
	core::Graph graph;

	core::Node* nodeA = graph.addNode(core::Node { "A" });
	core::Node* nodeB = graph.addNode(core::Node{ "B" });

	graph.addDirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

	REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
}

TEST_CASE("Create and check for weighted undirected edge", "[graph]") {
	core::Graph graph;

	core::Node* nodeA = graph.addNode(core::Node{ "A" });
	core::Node* nodeB = graph.addNode(core::Node{ "B" });

	graph.addUndirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

	REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
	REQUIRE(graph.existsEdge(nodeB->getIndex(), nodeA->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeB->getIndex(), nodeA->getIndex()) == 10);
}
