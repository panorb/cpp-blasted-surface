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

			float angle_bias = 50.0f;

			// If the angle bias is enabled, calculate the angle between the current node and the next node
			if (angle_bias > 0.0 && solution.size() >= 1)
			{
				Eigen::Vector3f current_node_position = points[current_node];
				Eigen::Vector3f next_node_position = points[i];
				Eigen::Vector3f previous_node_position = points[solution[solution.size() - 1]];

				Eigen::Vector3f current_to_next = next_node_position - current_node_position;
				Eigen::Vector3f current_to_previous = previous_node_position - current_node_position;

				current_to_next.normalize();
				current_to_previous.normalize();

				float angle = std::acos(current_to_next.dot(current_to_previous));

				distance += angle * angle_bias;
			}


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
