#pragma once
#include <vector>
#include <memory>
#include "node.h"

namespace blast {
	class Graph {
		std::vector<NodeP> nodes;
		// List of adjacency rows with pairs of weight and index of neighbor node
		std::vector<std::vector<std::pair<int, size_t>>> adjList;

	public:
		void addNode(NodeP node);
		NodeP getNode(size_t index);
		void addDirectedEdge(size_t from, size_t to, int weight);
		void addUndirectedEdge(size_t a, size_t b, int weight);
		int getEdgeWeight(size_t from, size_t to);
		bool existsEdge(size_t from, size_t to);
	};
};
