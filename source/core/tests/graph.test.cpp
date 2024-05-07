#include <graph.h>
#include <catch.hpp>

TEST_CASE("Graphs can be created and manipulated", "[graph]") {
	blast::Graph graph;

	SECTION("Adding nodes to a graph") {
		std::shared_ptr<blast::Node> node = std::make_shared<blast::Node>("Test");
		graph.addNode(node);
		REQUIRE(graph.getNode(0)->getLabel() == "Test");

		SECTION("Creating edge between the same node does not work") {
			graph.addDirectedEdge(node->getIndex(), node->getIndex(), 1);
			REQUIRE(graph.existsEdge(node->getIndex(), node->getIndex()) == false);
		}

		SECTION("Creating edge between non-existant node and existant node does not work") {
			graph.addDirectedEdge(node->getIndex(), 1, 1);
			REQUIRE(graph.existsEdge(node->getIndex(), 1) == false);
		}

		SECTION("Creating edge between two non-existant nodes does not work") {
			graph.addDirectedEdge(1, 2, 1);
			REQUIRE(graph.existsEdge(1, 2) == false);
		}
	}
	
	SECTION("Adding two nodes to a graph") {
		std::shared_ptr<blast::Node> nodeA = std::make_shared<blast::Node>("A");
		std::shared_ptr<blast::Node> nodeB = std::make_shared<blast::Node>("B");
		graph.addNode(nodeA);
		graph.addNode(nodeB);

		SECTION("Creating weighted directed edge") {
			graph.addDirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

			REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
			REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
		}

		SECTION("Creating weighted undirected edge") {
			graph.addUndirectedEdge(nodeA->getIndex(), nodeB->getIndex(), 10);

			REQUIRE(graph.existsEdge(nodeA->getIndex(), nodeB->getIndex()) == true);
			REQUIRE(graph.getEdgeWeight(nodeA->getIndex(), nodeB->getIndex()) == 10);
			REQUIRE(graph.existsEdge(nodeB->getIndex(), nodeA->getIndex()) == true);
			REQUIRE(graph.getEdgeWeight(nodeB->getIndex(), nodeA->getIndex()) == 10);
		}
	}

	SECTION("Creating graph with 100 connected nodes") {
		graph.addNode(std::make_shared<blast::Node>("Node 0"));

		for (int i = 1; i < 100; i++) {
			std::string label = "Node ";
			label += std::to_string(i);
			std::shared_ptr<blast::Node> node = std::make_shared<blast::Node>(label);
			graph.addNode(node);
			graph.addUndirectedEdge(i - 1, i, 1);
		}

		REQUIRE(graph.getNode(99)->getLabel() == "Node 99");
		REQUIRE(graph.existsEdge(98, 99) == true);
		REQUIRE(graph.getEdgeWeight(98, 99) == 1);
		REQUIRE(graph.existsEdge(99, 98) == true);
		REQUIRE(graph.getEdgeWeight(99, 98) == 1);
		REQUIRE(graph.getNodeCount() == 100);
	}
}
