#pragma once
#include <vector>
#include <memory>
#include "node.h"

namespace blast {
	class Graph {
		std::vector<std::shared_ptr<Node>> nodes;
		// List of adjacency rows with pairs of weight and index of neighbor node
		std::vector<std::vector<std::pair<int, size_t>>> adjList;

	public:
		void addNode(std::shared_ptr<Node> node);
		std::shared_ptr<Node> getNode(size_t index);
		void addDirectedEdge(size_t from, size_t to, int weight);
		void addUndirectedEdge(size_t a, size_t b, int weight);
		void removeDirectedEdge(size_t from, size_t to);
		void removeUndirectedEdge(size_t a, size_t b);
		int getEdgeWeight(size_t from, size_t to);
		bool existsEdge(size_t from, size_t to);
		int getNodeCount() { return nodes.size(); }
	};

}; // namespace blast
