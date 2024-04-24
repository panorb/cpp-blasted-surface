#include "graph.h"
#include <cassert>

core::Node* core::Graph::addNode(Node node)
{
	nodes.push_back(std::make_unique<Node>(node));
	nodes.back()->index = nodes.size() - 1;
	adjList.push_back(std::vector<std::pair<int, size_t>>());
	return nodes.back().get();
}

core::Node* core::Graph::getNode(size_t index)
{
	assert(index < nodes.size());

	return nodes[index].get();
}

void core::Graph::addDirectedEdge(size_t from, size_t to, int weight)
{
	assert(from < adjList.size());
	assert(to < adjList.size());

	adjList[from].push_back(std::make_pair(weight, to));
}

void core::Graph::addUndirectedEdge(size_t a, size_t b, int weight)
{
	addDirectedEdge(a, b, weight);
	addDirectedEdge(b, a, weight);
}

int core::Graph::getEdgeWeight(size_t from, size_t to)
{
	assert(from < nodes.size());
	assert(to < nodes.size());

	auto neighbors = adjList[from];

	for (auto neighbor : neighbors) {
		if (neighbor.second == to) {
			return neighbor.first;
		}
	}

	return 0;
}

bool core::Graph::existsEdge(size_t from, size_t to)
{
	return getEdgeWeight(from, to) != 0;
}
