#include "blast/graph.h"

void blast::Graph::add_node(std::shared_ptr<blast::Node> node)
{
	nodes.push_back(node);
	nodes.back()->index = nodes.size() - 1;
	adj_list.push_back(std::vector<std::pair<int, size_t>>());
}

void blast::Graph::remove_node(size_t index) {
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

std::shared_ptr<blast::Node> blast::Graph::get_node(size_t index)
{
	if (index >= nodes.size()) return nullptr;

	return nodes[index];
}
void blast::Graph::add_directed_edge(size_t from, size_t to, int weight)
{
	if (from == to || from >= adj_list.size() || to >= adj_list.size() || exists_edge(from, to)) {
		return;
	}

	adj_list[from].push_back(std::make_pair(weight, to));
}

void blast::Graph::add_undirected_edge(size_t a, size_t b, int weight)
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

int blast::Graph::get_edge_weight(size_t from, size_t to)
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

bool blast::Graph::exists_edge(size_t from, size_t to)
{
	return get_edge_weight(from, to) != 0;
}

int blast::Graph::get_node_count() const
{
	return nodes.size();
}
