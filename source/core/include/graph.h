#pragma once
#include <vector>
#include <memory>
#include "node.h"

namespace core {
	class Graph {
		std::vector<std::unique_ptr<Node>> nodes;
		// List of adjacency rows with pairs of weight and index of neighbor node
		std::vector<std::vector<std::pair<int, size_t>>> adjList;

	public:
		Node* addNode(Node node);
		Node* getNode(size_t index);
		void addDirectedEdge(size_t from, size_t to, int weight);
		void addUndirectedEdge(size_t a, size_t b, int weight);
		int getEdgeWeight(size_t from, size_t to);
		bool existsEdge(size_t from, size_t to);
	};
};