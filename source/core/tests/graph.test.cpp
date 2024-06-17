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
			REQUIRE_FALSE(graph.exists_edge(node->get_index(), node->get_index()));
		}

		SECTION("Creating edge between non-existant node and existant node does not work") {
			graph.add_directed_edge(node->get_index(), 1, 1);
			REQUIRE_FALSE(graph.exists_edge(node->get_index(), 1));
		}

		SECTION("Creating edge between two non-existant nodes does not work") {
			graph.add_directed_edge(1, 2, 1);
			REQUIRE_FALSE(graph.exists_edge(1, 2));
		}
	}
	
	SECTION("Adding two nodes to a graph") {
		auto node_a = std::make_shared<blast::Node>("A");
		auto node_b = std::make_shared<blast::Node>("B");
		graph.add_node(node_a);
		graph.add_node(node_b);

		SECTION("Creating weighted directed edge") {
			graph.add_directed_edge(node_a->get_index(), node_b->get_index(), 10);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()));
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10);
		}

		SECTION("Creating weighted undirected edge") {
			graph.add_undirected_edge(node_a->get_index(), node_b->get_index(), 10);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()));
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10);
			REQUIRE(graph.exists_edge(node_b->get_index(), node_a->get_index()));
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
		REQUIRE(graph.exists_edge(98, 99));
		REQUIRE(graph.get_edge_weight(98, 99) == 1);
		REQUIRE(graph.exists_edge(99, 98));
		REQUIRE(graph.get_edge_weight(99, 98) == 1);
		REQUIRE(graph.get_node_count() == 100);

		SECTION("Edges can be cleared") {
			graph.clear_edges();
			REQUIRE_FALSE(graph.exists_edge(0, 1));
			REQUIRE_FALSE(graph.exists_edge(99, 98));
		}
		
		SECTION("Graph can be cleared completely")
		{
			graph.clear();
			REQUIRE(graph.get_node_count() == 0);
			REQUIRE_FALSE(graph.exists_edge(0, 1));
		}

		SECTION("Graph can be copied and edges can be edited independently")
		{
			blast::Graph copy(graph);

			REQUIRE(copy.get_node(99)->get_label() == "Node 99");
			REQUIRE(copy.exists_edge(98, 99));
			REQUIRE(copy.get_edge_weight(98, 99) == 1);

			copy.remove_undirected_edge(98, 99);
			REQUIRE_FALSE(copy.exists_edge(98, 99));
			REQUIRE(graph.exists_edge(98, 99));

			copy.add_node(std::make_shared<blast::Node>("Node 100"));
			REQUIRE(copy.get_node_count() == 101);
			REQUIRE(graph.get_node_count() == 100);
		}
	}

	SECTION("Graph edges can be created via adjacency matrix")
	{
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
		REQUIRE(graph.get_edge_weight(0, 1) == 3);
		REQUIRE(graph.get_edge_weight(0, 2) == 4);
		REQUIRE_FALSE(graph.exists_edge(0, 3));
		REQUIRE(graph.get_edge_weight(0, 4) == 2);

		REQUIRE(graph.get_edge_weight(1, 0) == 3);
		REQUIRE(graph.get_edge_weight(1, 2) == 2);
		REQUIRE(graph.get_edge_weight(1, 3) == 5);
		REQUIRE(graph.get_edge_weight(1, 4) == 1);

		REQUIRE(graph.get_edge_weight(2, 0) == 4);
		REQUIRE(graph.get_edge_weight(2, 1) == 2);
		REQUIRE(graph.get_edge_weight(2, 3) == 1);
		REQUIRE(graph.get_edge_weight(2, 4) == 6);

		REQUIRE_FALSE(graph.exists_edge(3, 0));
		REQUIRE(graph.get_edge_weight(3, 1) == 5);
		REQUIRE(graph.get_edge_weight(3, 2) == 1);
		REQUIRE(graph.get_edge_weight(3, 4) == 3);

		REQUIRE(graph.get_edge_weight(4, 0) == 2);
		REQUIRE(graph.get_edge_weight(4, 1) == 1);
		REQUIRE(graph.get_edge_weight(4, 2) == 6);
		REQUIRE(graph.get_edge_weight(4, 3) == 3);
	}
}
