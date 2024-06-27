#pragma once

#include <vector>
#include <memory>
#include "node.hpp"

namespace blast {
	class Graph {
		std::vector<std::shared_ptr<Node>> nodes;
		// List of adjacency rows with pairs of weight and index of neighbor node
		std::vector<std::vector<std::pair<float, size_t>>> adj_list;
	public:
		Graph() = default;
		Graph(const Graph& original);

		void add_node(const std::shared_ptr<Node>& node);
		void remove_node(size_t index);
		std::shared_ptr<Node> get_node(size_t index) const;
		void add_directed_edge(size_t from, size_t to, float weight);
		void add_undirected_edge(size_t a, size_t b, float weight);
		void remove_directed_edge(size_t from, size_t to);
		void remove_undirected_edge(size_t a, size_t b);
		[[nodiscard]] float get_edge_weight(size_t from, size_t to) const;
		[[nodiscard]] bool exists_edge(size_t from, size_t to) const;
		[[nodiscard]] size_t get_node_count() const;
		void clear();
		void clear_edges();
	};

	float get_length_of_path(const Graph& graph, const std::vector<size_t>& path, bool loop);
	void add_edges_from_adjacency_matrix(Graph& graph, const std::vector<std::vector<float>>& adj_matrix);
}; // namespace blast
