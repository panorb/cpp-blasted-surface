#include "blast/greedy_optimizer.hpp"

#include <set>

std::vector<size_t> blast::Greedy_optimizer::execute()
{
	// Initialize the solution vector
	std::vector<size_t> solution;
	solution.reserve(node_graph->get_node_count());

	// Initialize the set of unvisited nodes
	std::set<size_t> unvisited_nodes;
	for (size_t i = 0; i < node_graph->get_node_count(); ++i)
	{
		unvisited_nodes.insert(i);
	}

	// Start at the first node
	size_t current_node = 0;
	solution.push_back(current_node);

	// Remove the first node from the unvisited set
	unvisited_nodes.erase(current_node);

	// While there are still unvisited nodes
	while (!unvisited_nodes.empty())
	{
		// Find the nearest unvisited node
		float min_distance = std::numeric_limits<float>::max();
		size_t nearest_node = 0;
		for (size_t i : unvisited_nodes)
		{
			float distance = node_graph->get_edge_weight(current_node, i).value_or(std::numeric_limits<float>::max());
			if (distance < min_distance)
			{
				min_distance = distance;
				nearest_node = i;
			}
		}

		// Add the nearest node to the solution
		solution.push_back(nearest_node);

		// Update the current node
		current_node = nearest_node;

		// Remove the nearest node from the unvisited set
		unvisited_nodes.erase(nearest_node);
	}

	return solution;
}
