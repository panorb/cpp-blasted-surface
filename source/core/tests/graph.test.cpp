#include <graph.h>
#include <catch.hpp>

TEST_CASE("Create and add nodes to graph", "[graph]") {
	blast::Graph graph;
	graph.addNode(std::make_shared<blast::Node>("Test"));

	REQUIRE(graph.getNode(0)->getLabel() == "Test");
}

TEST_CASE("Create and check for weighted directed edge", "[graph]") {
	blast::Graph graph;

	blast::NodeP nodeA = std::make_shared<blast::Node>("A");
	blast::NodeP nodeB = std::make_shared<blast::Node>("B");
	graph.addNode(nodeA);
	graph.addNode(nodeB);

	graph.addDirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

	REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
}

TEST_CASE("Create and check for weighted undirected edge", "[graph]") {
	blast::Graph graph;

	blast::NodeP nodeA = std::make_shared<blast::Node>("A");
	blast::NodeP nodeB = std::make_shared<blast::Node>("B");
	graph.addNode(nodeA);
	graph.addNode(nodeB);

	graph.addUndirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

	REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
	REQUIRE(graph.existsEdge(nodeB->getIndex(), nodeA->getIndex()) == true);
	REQUIRE(graph.getEdgeWeight(nodeB->getIndex(), nodeA->getIndex()) == 10);
}

TEST_CASE("Create graph with 100 unconnected nodes", "[graph]") {
	blast::Graph graph;

	for (int i = 0; i < 100; i++) {
		std::string label = "Node ";
		label += i;
		blast::NodeP node = std::make_shared<blast::Node>(label);
	}
}
