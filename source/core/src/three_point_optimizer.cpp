#include "blast/three_point_optimizer.hpp"

#include <set>

std::vector<size_t> blast::Three_point_optimizer::execute() const
{
	// Initialize the solution vector
	std::vector<size_t> solution;
	solution.reserve(node_graph->get_node_count());

	// Start at the first node
	solution.push_back(0);

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

	// Find the closes node to the first node
	size_t closest_node = 0;
	float closest_distance = std::numeric_limits<float>::max();

	for (size_t i = 0; i < node_graph->get_node_count(); ++i)
	{
		float cost = node_graph->get_edge_weight(0, i).value_or(std::numeric_limits<float>::max());
		if (cost < closest_distance)
		{
			closest_distance = cost;
			closest_node = i;
		}
	}

	// Add the closest node to the solution
	solution.push_back(closest_node);
	unvisited_nodes.erase(closest_node);

	while (!unvisited_nodes.empty())
	{
		// Until no nodes are left: Find the 3 closest sample points
		std::vector<size_t> closest_nodes = {};
		std::vector<float> closest_distances = {};
		int closest_amount = std::min(5, (int)unvisited_nodes.size()); // 3 Max, but less if there are less than 3 nodes left

		// Initialize the closest nodes and distances
		for (int i = 0; i < closest_amount; ++i)
		{
			closest_nodes.push_back(0);
			closest_distances.push_back(std::numeric_limits<float>::max());
		}

		// Find clost_amount of closest sample points
		for (size_t node_i : unvisited_nodes)
		{
			float cost = node_graph->get_edge_weight(solution[solution.size() - 1], node_i).value_or(std::numeric_limits<float>::max());

			for (int check = 0; check < closest_amount; ++check)
			{
				if (cost < closest_distances[check])
				{
					closest_distances.insert(closest_distances.begin() + check, cost);
					closest_nodes.insert(closest_nodes.begin() + check, node_i);
					break;
				}
			}
		}

		// Look which of the 3 closest sample points has the smallest angle to the last 2 sample points
		size_t best_node = 0;
		float best_angle = std::numeric_limits<float>::max();

		for (size_t i = 0; i < closest_amount; ++i)
		{
			Eigen::Vector3f point1 = points[solution[solution.size() - 2]];
			Eigen::Vector3f point2 = points[solution[solution.size() - 1]];
			Eigen::Vector3f point3 = points[closest_nodes[i]];

			// Calculate angle
			Eigen::Vector3f v1 = point1 - point2;
			Eigen::Vector3f v2 = point3 - point2;

			// Calculate dot product
			float dot = v1.dot(v2);

			// Calculate magnitudes
			float mag1 = v1.norm();
			float mag2 = v2.norm();

			// Calculate angle
			float angle = std::acos(dot / (mag1 * mag2));

			if (angle < best_angle)
			{
				best_angle = angle;
				best_node = closest_nodes[i];
			}
		}

		// Add the best node to the solution
		solution.push_back(best_node);
		unvisited_nodes.erase(best_node);
	}

	return solution;
}
