#include "blast/greedy_optimizer.hpp"

#include <set>
#include <spdlog/spdlog.h>

std::vector<size_t> blast::Greedy_optimizer::execute() const
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
	solution.push_back(0);

	// Remove the first node from the unvisited set
	unvisited_nodes.erase(0);

	// While there are still unvisited nodes
	while (!unvisited_nodes.empty())
	{

		// Find the nearest unvisited node
		float min_distance = std::numeric_limits<float>::max();
		size_t nearest_node = 0;

		for (size_t i : unvisited_nodes)
		{
			float cost = node_graph->get_edge_weight(solution[solution.size() - 1], i).value_or(std::numeric_limits<float>::max());

			float angle_bias = 50.0f;

			// If the angle bias is enabled, calculate the angle between the current node and the next node
			if (angle_bias > 0.0 && solution.size() >= 2)
			{
				Eigen::Vector3f previous_node_position = points[solution[solution.size() - 2]];
				Eigen::Vector3f current_node_position = points[solution[solution.size() - 1]];
				Eigen::Vector3f next_node_position = points[i];

				spdlog::info("== Path so far: ==");
				for (auto& st_pt : solution)
				{
					spdlog::info("({}, {}, {})", points[st_pt].x(), points[st_pt].y(), points[st_pt].z());
				}
				spdlog::info("==================");

				// Log the 3 positions to spdlog
				spdlog::info("Previous node position: {}, {}, {}", previous_node_position.x(), previous_node_position.y(), previous_node_position.z());
				spdlog::info("Current node position: {}, {}, {}", current_node_position.x(), current_node_position.y(), current_node_position.z());
				spdlog::info("Next node position: {}, {}, {}", next_node_position.x(), next_node_position.y(), next_node_position.z());

				Eigen::Vector3f current_to_next = next_node_position - current_node_position;
				Eigen::Vector3f current_to_previous = previous_node_position - current_node_position;

				current_to_next.normalize();
				current_to_previous.normalize();

				float dot_product = current_to_next.dot(current_to_previous);

				spdlog::info("Current to next: {}, {}, {}", current_to_next.x(), current_to_next.y(), current_to_next.z());
				spdlog::info("Current to previous: {}, {}, {}", current_to_previous.x(), current_to_previous.y(), current_to_previous.z());

				spdlog::info("Punktprodukt (inv.): {}", 1.0f - dot_product);

				cost += (1.0f - dot_product) * angle_bias;
			}


			if (cost < min_distance)
			{
				min_distance = cost;
				nearest_node = i;
			}
		}

		// Add the nearest node to the solution
		solution.push_back(nearest_node);

		// Remove the nearest node from the unvisited set
		unvisited_nodes.erase(nearest_node);
	}

	return solution;
}
