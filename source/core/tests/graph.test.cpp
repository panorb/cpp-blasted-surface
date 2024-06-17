#include <blast/graph.hpp>
#include <catch.hpp>

TEST_CASE("Graphs can be created and manipulated", "[graph]") {
	blast::Graph graph;
	REQUIRE(graph.get_node_count() == 0);

	SECTION("Adding nodes to a graph") {
		auto node = std::make_shared<blast::Node>("Test");
		graph.add_node(node);
		REQUIRE(graph.get_node(0)->get_label() == "Test");

		SECTION("Creating edge between the same node does not work") {
			graph.add_directed_edge(node->get_index(), node->get_index(), 1);
			REQUIRE(graph.exists_edge(node->get_index(), node->get_index()) == false);
		}

		SECTION("Creating edge between non-existant node and existant node does not work") {
			graph.add_directed_edge(node->get_index(), 1, 1);
			REQUIRE(graph.exists_edge(node->get_index(), 1) == false);
		}

		SECTION("Creating edge between two non-existant nodes does not work") {
			graph.add_directed_edge(1, 2, 1);
			REQUIRE(graph.exists_edge(1, 2) == false);
		}
	}
	
	SECTION("Adding two nodes to a graph") {
		auto node_a = std::make_shared<blast::Node>("A");
		auto node_b = std::make_shared<blast::Node>("B");
		graph.add_node(node_a);
		graph.add_node(node_b);

		SECTION("Creating weighted directed edge") {
			graph.add_directed_edge(node_a->get_index(), node_b->get_index(), 10);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()) == true);
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10);
		}

		SECTION("Creating weighted undirected edge") {
			graph.add_undirected_edge(node_a->get_index(), node_b->get_index(), 10);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()) == true);
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10);
			REQUIRE(graph.exists_edge(node_b->get_index(), node_a->get_index()) == true);
			REQUIRE(graph.get_edge_weight(node_b->get_index(), node_a->get_index()) == 10);
		}
	}

	SECTION("Creating graph with 100 connected nodes") {
		graph.add_node(std::make_shared<blast::Node>("Node 0"));

		for (int i = 1; i < 100; i++) {
			std::string label = "Node ";
			label += std::to_string(i);
			auto node = std::make_shared<blast::Node>(label);
			graph.add_node(node);
			graph.add_undirected_edge(i - 1, i, 1);
		}

		REQUIRE(graph.get_node(99)->get_label() == "Node 99");
		REQUIRE(graph.exists_edge(98, 99) == true);
		REQUIRE(graph.get_edge_weight(98, 99) == 1);
		REQUIRE(graph.exists_edge(99, 98) == true);
		REQUIRE(graph.get_edge_weight(99, 98) == 1);
		REQUIRE(graph.get_node_count() == 100);
	}
}
