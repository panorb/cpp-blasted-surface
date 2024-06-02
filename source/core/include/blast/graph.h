#pragma once

#include <vector>
#include <memory>
#include "node.h"

namespace blast {
	class Graph {
		std::vector<std::shared_ptr<Node>> nodes;
		// List of adjacency rows with pairs of weight and index of neighbor node
		std::vector<std::vector<std::pair<int, size_t>>> adj_list;

	public:
		void add_node(std::shared_ptr<Node> node);
		void remove_node(size_t index);
		std::shared_ptr<Node> get_node(size_t index);
		void add_directed_edge(size_t from, size_t to, int weight);
		void add_undirected_edge(size_t a, size_t b, int weight);
		void remove_directed_edge(size_t from, size_t to);
		void remove_undirected_edge(size_t a, size_t b);
		int get_edge_weight(size_t from, size_t to);
		bool exists_edge(size_t from, size_t to);
		int get_node_count() const;
	};

}; // namespace blast
