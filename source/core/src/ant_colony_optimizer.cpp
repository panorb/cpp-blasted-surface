#include "blast/ant_colony_optimizer.hpp"

std::vector<size_t> blast::Ant_colony_optimizer::execute(const blast::Graph& node_graph)
{
	blast::Graph pheromone_graph{ node_graph }; // This copy of the graph will store the pheromone values
	pheromone_graph.clear_edges(); // Clear all edges from the graph

	std::vector<size_t> best_solution;

	for (int i = 0; i < iteration_count; i++)
	{
		// Do we have to check if the solution is better than the current best solution?
		// Or can we just assume that the solution is better?
		best_solution = construct_solution(pheromone_graph, node_graph);
		global_pheromone_update(pheromone_graph, best_solution);
		evaporate_pheromone(pheromone_graph);
	}

	return best_solution;
}

std::vector<size_t> blast::Ant_colony_optimizer::construct_solution(const blast::Graph& pheromone_graph, const blast::Graph& node_graph) const
{
	size_t node_count = node_graph.get_node_count();

	std::vector<size_t> best_solution;
	int best_solution_const = INT_MAX;

	// TODO: This could totally be parallelized
	for (int i = 0; i < ant_count; i++)
	{
		std::vector<size_t> solution;
		solution.reserve(node_count);

		for (size_t j = 0; j < node_count; j++)
		{
			// Pick next node randomly
			// TODO: Select random node more intelligently
			// TODO: Use the actual pheromone values

			// pheromone_graph


			auto node = node_graph.get_node(rand() % node_count);
			solution.push_back(node->get_index());
		}

		int solution_cost = blast::get_length_of_path(node_graph, solution, true);
		if (solution_cost < best_solution_const)
		{
			best_solution = solution;
			best_solution_const = solution_cost;
		}
	}

	return best_solution;
}

void blast::Ant_colony_optimizer::global_pheromone_update(blast::Graph& pheromone_graph, const std::vector<size_t>& best_solution)
{
	// TODO: Implement this
}

void blast::Ant_colony_optimizer::evaporate_pheromone(blast::Graph& pheromone_graph)
{
	for (size_t i = 0; i < pheromone_graph.get_node_count(); i++)
	{
		for (size_t j = 0; j < pheromone_graph.get_node_count(); j++)
		{
			float edge_weight = pheromone_graph.get_edge_weight(i, j);
			if (edge_weight != -1) // pheromone_graph.exists_edge(i, j);
			{
				// Evaporate pheromone
				edge_weight *= 0.9;
				// Recreate edge with new pheromone amount
				pheromone_graph.remove_directed_edge(i, j);
				pheromone_graph.add_directed_edge(i, j, edge_weight);
			}
		}
	}
}
