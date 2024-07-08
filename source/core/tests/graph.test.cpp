#include <blast/graph.hpp>
#include <catch.hpp>

#include "blast/log.hpp"

TEST_CASE("Graphs can be created and manipulated", "[graph]")
{
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
			graph.add_directed_edge(node_a->get_index(), node_b->get_index(), 10.f);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()));
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10.f);
		}

		SECTION("Creating weighted undirected edge") {
			graph.add_undirected_edge(node_a->get_index(), node_b->get_index(), 10);

			REQUIRE(graph.exists_edge(node_a->get_index(), node_b->get_index()));
			REQUIRE(graph.get_edge_weight(node_a->get_index(), node_b->get_index()) == 10.f);
			REQUIRE(graph.exists_edge(node_b->get_index(), node_a->get_index()));
			REQUIRE(graph.get_edge_weight(node_b->get_index(), node_a->get_index()) == 10.f);
		}
	}

	SECTION("Length of path is correctly calculated")
	{
		graph.add_node(std::make_shared<blast::Node>("Node A"));
		graph.add_node(std::make_shared<blast::Node>("Node B"));
		graph.add_node(std::make_shared<blast::Node>("Node C"));

		graph.add_directed_edge(0, 1, 3);
		graph.add_directed_edge(1, 2, 3);
		graph.add_directed_edge(2, 0, 3);

		std::vector<size_t> path = { 0, 1, 2 };

		auto path_length = blast::get_length_of_path(graph, path, true);
		REQUIRE(path_length.has_value());
		REQUIRE(*path_length == 9.f);

		path = { 2, 1, 0 };
		path_length = blast::get_length_of_path(graph, path, true);
		REQUIRE(!path_length.has_value());
	}

	SECTION("Creating graph with 100 connected nodes") {
		graph.add_node(std::make_shared<blast::Node>("Node 0"));

		for (int i = 1; i < 100; i++) {
			std::string label = "Node ";
			label += std::to_string(i);
			auto node = std::make_shared<blast::Node>(label);
			graph.add_node(node);
			graph.add_undirected_edge(i - 1, i, 1.f);
		}

		REQUIRE(graph.get_node(99)->get_label() == "Node 99");
		REQUIRE(graph.exists_edge(98, 99));
		REQUIRE(graph.get_edge_weight(98, 99) == 1.f);
		REQUIRE(graph.exists_edge(99, 98));
		REQUIRE(graph.get_edge_weight(99, 98) == 1.f);
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
			REQUIRE(copy.get_edge_weight(98, 99) == 1.f);

			copy.remove_undirected_edge(98, 99);
			REQUIRE_FALSE(copy.exists_edge(98, 99));
			REQUIRE(graph.exists_edge(98, 99));

			copy.add_node(std::make_shared<blast::Node>("Node 100"));
			REQUIRE(copy.get_node_count() == 101);
			REQUIRE(graph.get_node_count() == 100);
		}

		SECTION("Removing nodes from graph corrects indices accordingly")
		{
			graph.remove_node(50);
			REQUIRE(graph.get_node_count() == 99);
			REQUIRE(graph.get_node(50)->get_index() == 50);
		}
	}

	SECTION("Graph edges can be created via adjacency matrix")
	{
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
		REQUIRE(graph.get_edge_weight(0, 1).value_or(0.f) == 3.f);
		REQUIRE(graph.get_edge_weight(0, 2).value_or(0.f) == 4.f);
		REQUIRE_FALSE(graph.exists_edge(0, 3));
		REQUIRE(graph.get_edge_weight(0, 4).value_or(0.f) == 2.f);

		REQUIRE(graph.get_edge_weight(1, 0).value_or(0.f) == 3.f);
		REQUIRE(graph.get_edge_weight(1, 2).value_or(0.f) == 2.f);
		REQUIRE(graph.get_edge_weight(1, 3).value_or(0.f) == 5.f);
		REQUIRE(graph.get_edge_weight(1, 4).value_or(0.f) == 1.f);

		REQUIRE(graph.get_edge_weight(2, 0).value_or(0.f) == 4.f);
		REQUIRE(graph.get_edge_weight(2, 1).value_or(0.f) == 2.f);
		REQUIRE(graph.get_edge_weight(2, 3).value_or(0.f) == 1.f);
		REQUIRE(graph.get_edge_weight(2, 4).value_or(0.f) == 6.f);

		REQUIRE_FALSE(graph.exists_edge(3, 0));
		REQUIRE(graph.get_edge_weight(3, 1).value_or(0.f) == 5.f);
		REQUIRE(graph.get_edge_weight(3, 2).value_or(0.f) == 1.f);
		REQUIRE(graph.get_edge_weight(3, 4).value_or(0.f) == 3.f);

		REQUIRE(graph.get_edge_weight(4, 0).value_or(0.f) == 2.f);
		REQUIRE(graph.get_edge_weight(4, 1).value_or(0.f) == 1.f);
		REQUIRE(graph.get_edge_weight(4, 2).value_or(0.f) == 6.f);
		REQUIRE(graph.get_edge_weight(4, 3).value_or(0.f) == 3.f);
	}
}
