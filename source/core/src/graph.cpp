#include "blast/graph.hpp"

blast::Graph::Graph(const Graph& original)
{
	nodes = original.nodes;
	adj_list = original.adj_list;
}

void blast::Graph::add_node(const std::shared_ptr<blast::Node>& node)
{
	nodes.push_back(node);
	nodes.back()->index = nodes.size() - 1;
	adj_list.push_back(std::vector<std::pair<int, size_t>>());
}

void blast::Graph::remove_node(const size_t index) {
	if (index >= nodes.size()) return;

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
}

std::shared_ptr<blast::Node> blast::Graph::get_node(size_t index) const
{
	if (index >= nodes.size()) return nullptr;

	return nodes[index];
}

void blast::Graph::add_directed_edge(const size_t from, const size_t to, const int weight)
{
	if (from == to || from >= adj_list.size() || to >= adj_list.size() || exists_edge(from, to)) {
		return;
	}

	adj_list[from].push_back(std::make_pair(weight, to));
}

void blast::Graph::add_undirected_edge(const size_t a, const size_t b, const int weight)
{
	add_directed_edge(a, b, weight);
	add_directed_edge(b, a, weight);
}

void blast::Graph::remove_directed_edge(size_t from, size_t to) {
	auto& neighbors = adj_list[from];

	for (auto it = neighbors.begin(); it != neighbors.end(); it++) {
		if (it->second == to) {
			neighbors.erase(it);
			return;
		}
	}
}

void blast::Graph::remove_undirected_edge(size_t a, size_t b)
{
	remove_directed_edge(a, b);
	remove_directed_edge(b, a);
}

int blast::Graph::get_edge_weight(const size_t from, const size_t to) const
{
	if (from >= nodes.size() || to >= nodes.size()) return 0;

	auto& neighbors = adj_list[from];

	for (auto& neighbor : neighbors) {
		if (neighbor.second == to) {
			return neighbor.first;
		}
	}

	return 0;
}

bool blast::Graph::exists_edge(const size_t from, const size_t to) const
{
	return get_edge_weight(from, to) != 0;
}

size_t blast::Graph::get_node_count() const
{
	return nodes.size();
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

int blast::get_length_of_path(const Graph& graph, const std::vector<size_t>& path, const bool loop)
{
	int length = 0;

	for (size_t i = 0; i < path.size() - 1; i++) {
		length += graph.get_edge_weight(path[i], path[i + 1]);
	}

	if (loop)
	{
		length += graph.get_edge_weight(path.back(), path.front());
	}

	return length;
}
