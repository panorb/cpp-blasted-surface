#include "blast/graph.hpp"
#include <spdlog/spdlog.h>

blast::Graph::Graph(const Graph& original)
{
	// Node references are kept intact - ownership handled by shared_ptr
	nodes = original.nodes;
	adj_list = original.adj_list;
}

void blast::Graph::add_node(const std::shared_ptr<blast::Node>& node)
{
	nodes.push_back(node);
	nodes.back()->index = nodes.size() - 1;
	adj_list.push_back(std::vector<std::pair<float, size_t>>());
}

void blast::Graph::remove_node(const size_t index) {
	spdlog::trace("Removing node {}", index);
	if (index >= nodes.size())
	{
		spdlog::warn("Removing node {} failed, node does not exist", index);
		return;
	}

	nodes.erase(nodes.begin() + index);
	adj_list.erase(adj_list.begin() + index);

	for (auto& neighbors : adj_list) {
		for (auto it = neighbors.begin(); it != neighbors.end(); it++) {
			if (it->second == index) {
				neighbors.erase(it);
				break;
			}
		}
	}

	for (auto& neighbors : adj_list) {
		for (auto& neighbor : neighbors) {
			if (neighbor.second > index) {
				neighbor.second--;
			}
		}
	}

	// Correct indices of all following nodes
	for (size_t i = index; i < nodes.size(); i++) {
		nodes[i]->index = i;
	}
}

std::shared_ptr<blast::Node> blast::Graph::get_node(size_t index) const
{
	if (index >= nodes.size()) {
		spdlog::warn("Getting node {} failed, out-of-bounds. Returning nullptr...", index);
		return nullptr;
	}

	return nodes[index];
}

void blast::Graph::add_directed_edge(const size_t from, const size_t to, const float weight)
{
	spdlog::trace("Adding directed edge from Node {} to Node {}", from, to);
	if (from == to || from >= adj_list.size() || to >= adj_list.size() || exists_edge(from, to)) {
		spdlog::warn("Adding the edge was aborted, either one of the nodes does not exist, it is the same node twice or the edge already exists");
		return;
	}

	adj_list[from].push_back(std::make_pair(weight, to));
}

void blast::Graph::add_undirected_edge(const size_t a, const size_t b, const float weight)
{
	add_directed_edge(a, b, weight);
	add_directed_edge(b, a, weight);
}

void blast::Graph::add_or_update_directed_edge(size_t from, size_t to, float weight)
{
	remove_directed_edge(from, to);
	add_directed_edge(from, to, weight);
}

void blast::Graph::add_or_update_undirected_edge(size_t a, size_t b, float weight)
{
	add_or_update_directed_edge(a, b, weight);
	add_or_update_directed_edge(b, a, weight);
}

void blast::Graph::remove_directed_edge(size_t from, size_t to) {
	spdlog::trace("Removing directed edge from Node {} to Node {}", from, to);
	auto& neighbors = adj_list[from];

	for (auto it = neighbors.begin(); it != neighbors.end(); it++) {
		if (it->second == to) {
			neighbors.erase(it);
			return;
		}
	}

	spdlog::warn("Removing the edge failed, edge does not exist.");
}

void blast::Graph::remove_undirected_edge(size_t a, size_t b)
{
	remove_directed_edge(a, b);
	remove_directed_edge(b, a);
}

std::optional<float> blast::Graph::get_edge_weight(const size_t from, const size_t to) const
{
	if (from >= nodes.size() || to >= nodes.size()) return {};

	const auto& neighbors = adj_list[from];

	for (const auto& neighbor : neighbors) {
		if (neighbor.second == to) {
			return neighbor.first;
		}
	}

	return {};
}

bool blast::Graph::exists_edge(const size_t from, const size_t to) const
{
	return get_edge_weight(from, to).has_value();
}

size_t blast::Graph::get_node_count() const
{
	return nodes.size();
}

std::vector<std::pair<float, size_t>> blast::Graph::get_connected_nodes(const size_t from) const
{
	spdlog::trace("Getting connected nodes of node {}", from);
	return adj_list[from];
}

void blast::Graph::clear()
{
	nodes.clear();
	clear_edges();
}

void blast::Graph::clear_edges()
{
	
	adj_list.clear();
	adj_list.resize(nodes.size());
}

std::optional<float> blast::get_length_of_path(const Graph& graph, const std::vector<size_t>& path, const bool loop)
{
	float length = 0.f;

	if (path.size() <= 1) return 0.f;

	for (size_t i = 0; i < path.size() - 1; i++) {
		const auto edge_weight = graph.get_edge_weight(path[i], path[i + 1]);
		if (!edge_weight.has_value()) return {}; // Invalid path
		length += *edge_weight;
	}

	if (loop)
	{
		const auto edge_weight = graph.get_edge_weight(path.back(), path.front());
		if (!edge_weight.has_value()) return {}; // Invalid path
		length += *edge_weight;
	}

	return length;
}

void blast::add_edges_from_adjacency_matrix(Graph& graph, const std::vector<std::vector<float>>& adj_matrix)
{
	for (int i = 0; i < adj_matrix.size(); i++)
	{
		for (int j = 0; j < adj_matrix[i].size(); j++)
		{
			if (adj_matrix[i][j] != 0.f)
			{
				graph.add_directed_edge(i, j, adj_matrix[i][j]);
			}
		}
	}
}
