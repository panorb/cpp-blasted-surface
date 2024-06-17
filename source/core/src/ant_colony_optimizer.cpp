#include "blast/ant_colony_optimizer.hpp"

std::vector<size_t> blast::AntColonyOptimizer::execute(blast::Graph& node_graph)
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

std::vector<size_t> blast::AntColonyOptimizer::construct_solution(const blast::Graph& pheromone_graph, const blast::Graph& node_graph) const
{
	size_t node_count = node_graph.get_node_count();

	std::vector<size_t> best_solution;
	int best_solution_length = INT_MAX;

	for (int i = 0; i < ant_count; i++)
	{
		std::vector<size_t> solution;
		solution.reserve(node_count);

		for (size_t j = 0; j < node_count; j++)
		{
			// Pick next node randomly
			// TODO: Select random node more intelligently
			// TODO: Use the actual pheromone values
			auto node = node_graph.get_node(rand() % node_count);
			solution.push_back(node->get_index());
		}

		int solution_length = blast::get_length_of_path(node_graph, solution, true);
		if (solution_length < best_solution_length)
		{
			best_solution = solution;
			best_solution_length = solution_length;
		}
	}

	return best_solution;
}

void blast::AntColonyOptimizer::global_pheromone_update(blast::Graph& pheromone_graph, const std::vector<size_t>& best_solution)
{
	// TODO: Implement this
}

void blast::AntColonyOptimizer::evaporate_pheromone(blast::Graph& pheromone_graph)
{
	// TODO: Implement this
}
