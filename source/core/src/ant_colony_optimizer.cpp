#include "blast/ant_colony_optimizer.hpp"

#include <chrono>
#include <random>
#include <unordered_set>

std::vector<size_t> blast::Ant_colony_optimizer::execute(const blast::Graph& node_graph)
{
	blast::Graph pheromone_graph{ node_graph }; // This copy of the graph will store the pheromone values
	// pheromone_graph.clear_edges(); // Clear all edges from the graph

	std::vector<std::vector<size_t>> solutions;

	for (int i = 0; i < iteration_count; i++)
	{
		// Do we have to check if the solution is better than the current best solution?
		// Or can we just assume that the solution is better?
		solutions = construct_solution(pheromone_graph, node_graph);
		global_pheromone_update(node_graph, pheromone_graph, solutions);
		evaporate_pheromone(pheromone_graph);
	}

	return solutions[0];
}

std::vector<std::vector<size_t>> blast::Ant_colony_optimizer::construct_solution(
	const blast::Graph& pheromone_graph, const blast::Graph& node_graph) const
{
	size_t node_count = node_graph.get_node_count();

	std::vector<std::vector<size_t>> solutions;
	float best_solution_const = std::numeric_limits<float>::max();

	for (int i = 0; i < ant_count; i++)
	{
		std::unordered_set<size_t> visited;
		std::vector<size_t> solution;

		solution.reserve(node_count);

		// Pick first node randomly
		unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();

		size_t first_node = 0;
		std::mt19937 generator(seed); // Standard mersenne_twister_engine seeded with rd()
		{
			std::uniform_int_distribution<size_t> distribution(0, node_count - 1);
			first_node = distribution(generator);
			solution.push_back(first_node);
			visited.insert(first_node);
		}


		for (size_t j = 0; j < node_count; j++)
		{
			// Pick next node randomly
			// TODO: Select random node more intelligently
			// TODO: Use the actual pheromone values
			auto connected_nodes = pheromone_graph.get_connected_nodes(solution.back());
			for (auto it = connected_nodes.begin(); it != connected_nodes.end();)
			{
				if (visited.contains(it->second))
				{
					it = connected_nodes.erase(it);
				}
				else
				{
					++it;
				}
			}

			std::vector<float> pheromone_levels;
			for (const auto& node : connected_nodes)
			{
				if (visited.contains(node.second))
				{
					pheromone_levels.push_back(node.first);
				}
			}

			std::discrete_distribution<int> distribution{ pheromone_levels.begin(), pheromone_levels.end() };

			int res = distribution(generator);

			if (res == 0 && res >= connected_nodes.size())
			{
				j = 0;
				solution.clear();
				visited.clear();
				solution.push_back(first_node);
				visited.insert(first_node);
				continue;
			}
			solution.push_back(connected_nodes[res].second);
			visited.insert(connected_nodes[res].second);
		}

		solutions.push_back(solution);
	}

	return solutions;
}

void blast::Ant_colony_optimizer::global_pheromone_update(const blast::Graph& node_graph, blast::Graph& pheromone_graph, const std::vector<std::vector<size_t>>& taken_paths)
{
	for (const auto& path : taken_paths)
	{
		float path_length = *blast::get_length_of_path(node_graph, path, true);
		float pheromone_per_edge = 20.0f / path_length;

		for (size_t i = 0; i < path.size() - 1; i++)
		{
			size_t from = path[i];
			size_t to = path[i + 1];

			float edge_weight = *pheromone_graph.get_edge_weight(from, to);
			edge_weight += pheromone_per_edge;

			pheromone_graph.remove_directed_edge(from, to);
			pheromone_graph.add_directed_edge(from, to, edge_weight);
		}
	}
}

void blast::Ant_colony_optimizer::evaporate_pheromone(blast::Graph& pheromone_graph)
{
	for (size_t i = 0; i < pheromone_graph.get_node_count(); i++)
	{
		for (size_t j = 0; j < pheromone_graph.get_node_count(); j++)
		{
			if (auto edge_weight = pheromone_graph.get_edge_weight(i, j); edge_weight.has_value()) // pheromone_graph.exists_edge(i, j);
			{
				float edge_weight_val = *edge_weight;
				// Evaporate pheromone
				edge_weight_val *= 0.9;
				// Recreate edge with new pheromone amount
				pheromone_graph.remove_directed_edge(i, j);
				pheromone_graph.add_directed_edge(i, j, edge_weight_val);
			}
		}
	}
}
