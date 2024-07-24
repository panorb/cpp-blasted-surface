#include "blast/ant_colony_optimizer.hpp"

#include <chrono>
#include <random>
#include <unordered_set>
#include <spdlog/spdlog.h>

void ensure_complete_graph(const blast::Graph& graph)
{
	for (size_t i = 0; i < graph.get_node_count(); i++)
	{
		for (size_t j = 0; j < graph.get_node_count(); j++)
		{
			if (i != j && !graph.exists_edge(i, j))
			{
				throw std::runtime_error("Graph is not complete");
			}
		}
	}
}

blast::Ant_colony_optimizer::Ant_colony_optimizer(const Graph* node_graph, const int ant_count,
	const int iteration_count, const float evaporation_rate, const float alpha, const float beta, const float pheromone_constant) : node_graph(node_graph), ant_count(ant_count), evaporation_rate(evaporation_rate), alpha(alpha), beta(beta), pheromone_constant(pheromone_constant)
{
	pheromone_graph = std::make_unique<Graph>(*node_graph); // This copy of the graph will store the pheromone values

	// Change all pheromone values (stored in the graph edges) to 1.0f
	for (size_t i = 0; i < pheromone_graph->get_node_count(); i++)
	{
		for (size_t j = 0; j < pheromone_graph->get_node_count(); j++)
		{
			pheromone_graph->add_or_update_directed_edge(i, j, 1.f);
		}
	}
}

blast::Ant_colony_optimizer_iteration_result blast::Ant_colony_optimizer::execute_iteration()
{
	std::vector<std::vector<size_t>> solutions = construct_solutions();
	global_pheromone_update(solutions);
	evaporate_pheromone();

	std::vector<size_t> best_solution_this_iteration = solutions[0];
	float best_solution_this_iteration_length = blast::get_length_of_path(*node_graph, best_solution_this_iteration, true).value_or(std::numeric_limits<float>::max());

	for (size_t i = 0; i < solutions.size(); i++)
	{
		float solution_length = blast::get_length_of_path(*node_graph, solutions[i], true).value_or(std::numeric_limits<float>::max());

		if (solution_length < best_solution_this_iteration_length)
		{
			best_solution_this_iteration = solutions[i];
			best_solution_this_iteration_length = solution_length;
		}
	}

	if (best_solution_this_iteration_length < blast::get_length_of_path(*node_graph, best_overall_solution, true).value_or(std::numeric_limits<float>::max()))
	{
		best_overall_solution = best_solution_this_iteration;
	}

	return { 0, best_solution_this_iteration, best_overall_solution, *pheromone_graph };
}



std::vector<size_t> blast::Ant_colony_optimizer::execute_iterations(size_t iteration_count)
{
	std::vector<std::vector<size_t>> solutions;

	for (int i = 0; i < iteration_count; i++)
	{
		execute_iteration();
	}

	return best_overall_solution;
}

void blast::Ant_colony_optimizer::initialize()
{
	// ensure_complete_graph(node_graph);
}


std::vector<std::vector<size_t>> blast::Ant_colony_optimizer::construct_solutions() const
{
	size_t node_count = node_graph->get_node_count();

	std::vector<std::vector<size_t>> solutions;
	float best_solution_const = std::numeric_limits<float>::max();

	std::unordered_set<size_t> used_starts;
	if (used_starts.size() == node_count)
	{
		used_starts.clear();
	}

	// TODO: This is probably parallelizable
	for (int i = 0; i < ant_count; i++)
	{
		std::unordered_set<size_t> visited;
		std::vector<size_t> solution;

		solution.reserve(node_count);

		// Pick first node completely - and independently - randomly
		unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();

		size_t first_node = 0;
		std::mt19937 generator(seed); // Standard mersenne_twister_engine seeded with rd()
		{
			do {
				std::uniform_int_distribution<size_t> distribution(0, node_count - 1);
				first_node = distribution(generator);
			} while (used_starts.contains(first_node));

			solution.push_back(first_node);
			visited.insert(first_node);
			used_starts.insert(first_node);
		}

		for (size_t j = 1; j < node_count; j++)
		{
			// Find all nodes that are connected to the last node in the solution
			auto connected_nodes = pheromone_graph->get_connected_nodes(solution.back());
			// Filter out those that have already been visited
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

			// Find the pheromone levels of the considered nodes
			std::vector<float> pheromone_levels;
			for (const auto& node : connected_nodes)
			{
				pheromone_levels.push_back(node.first);
			}

			std::vector<float> actual_distances;
			actual_distances.reserve(connected_nodes.size());
			for (const auto& node : connected_nodes)
			{
				actual_distances.push_back(node_graph->get_edge_weight(solution.back(), node.second).value_or(std::numeric_limits<float>::max()));
			}

			std::vector<float> probabilities;
			probabilities.reserve(connected_nodes.size());
			for (size_t k = 0; k < connected_nodes.size(); k++)
			{
				float pheromone = pheromone_levels[k];
				float distance = actual_distances[k];
				float probability = pow(pheromone, alpha) * pow(1.f / distance, beta);
				probabilities.push_back(probability);
			}

			std::discrete_distribution<int> distribution{ pheromone_levels.begin(), pheromone_levels.end() };

			int res = distribution(generator);

			solution.push_back(connected_nodes[res].second);
			visited.insert(connected_nodes[res].second);
		}

		solutions.push_back(solution);
	}

	return solutions;
}

void blast::Ant_colony_optimizer::global_pheromone_update(const std::vector<std::vector<size_t>>& taken_paths)
{
	for (const auto& path : taken_paths)
	{
		float path_length = blast::get_length_of_path(*node_graph, path, true).value_or(std::numeric_limits<float>::max());
		float pheromone_per_edge = pheromone_constant / path_length;

		for (size_t i = 0; i < path.size() - 1; i++)
		{
			size_t from = path[i];
			size_t to = path[i + 1];

			float edge_weight = *pheromone_graph->get_edge_weight(from, to);
			edge_weight += pheromone_per_edge;

			pheromone_graph->add_or_update_undirected_edge(from, to, edge_weight);
		}

		float edge_weight = *pheromone_graph->get_edge_weight(path[path.size() - 1], path[0]);
		edge_weight += pheromone_per_edge;

		pheromone_graph->add_or_update_undirected_edge(path[path.size() - 1], path[0], edge_weight);
	}
}

void blast::Ant_colony_optimizer::evaporate_pheromone()
{
	for (size_t i = 0; i < pheromone_graph->get_node_count(); i++)
	{
		for (size_t j = 0; j < pheromone_graph->get_node_count(); j++)
		{
			if (i != j)
			{
				auto edge_weight = *pheromone_graph->get_edge_weight(i, j); // pheromone_graph.exists_edge(i, j);

				// Evaporate pheromone
				edge_weight *= evaporation_rate;
				// Recreate edge with new pheromone amount
				pheromone_graph->add_or_update_directed_edge(i, j, edge_weight);
			}
		}
	}
}
