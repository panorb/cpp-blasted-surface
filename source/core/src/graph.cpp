#include "graph.h"

void blast::Graph::addNode(NodeP node)
{
	nodes.push_back(node);
	nodes.back()->index = nodes.size() - 1;
	adjList.push_back(std::vector<std::pair<int, size_t>>());
}

blast::NodeP blast::Graph::getNode(size_t index)
{
	if (index >= nodes.size()) return nullptr;

	return nodes[index];
}

void blast::Graph::addDirectedEdge(size_t from, size_t to, int weight)
{
	if (from == to || from >= adjList.size() || to >= adjList.size() || existsEdge(from, to)) {
		return;
	}

	adjList[from].push_back(std::make_pair(weight, to));
}

void blast::Graph::addUndirectedEdge(size_t a, size_t b, int weight)
{
	addDirectedEdge(a, b, weight);
	addDirectedEdge(b, a, weight);
}

int blast::Graph::getEdgeWeight(size_t from, size_t to)
{
	if (from >= nodes.size() || to >= nodes.size()) return 0;

	auto neighbors = adjList[from];

	for (auto neighbor : neighbors) {
		if (neighbor.second == to) {
			return neighbor.first;
		}
	}

	return 0;
}

bool blast::Graph::existsEdge(size_t from, size_t to)
{
	return getEdgeWeight(from, to) != 0;
}
