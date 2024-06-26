#pragma once

#include "graph.hpp"

namespace blast {
	class Ant_colony_optimizer
	{
	public:
		Ant_colony_optimizer(const int ant_count, const int iteration_count) : ant_count(ant_count), iteration_count(iteration_count) {}
		int ant_count;
		int iteration_count;

		std::vector<size_t> execute(blast::Graph& node_graph);
	private:
		std::vector<size_t> construct_solution(const blast::Graph& pheromone_graph, const blast::Graph& node_graph) const;
		void global_pheromone_update(blast::Graph& pheromone_graph, const std::vector<size_t>& best_solution);
		void evaporate_pheromone(blast::Graph& pheromone_graph);
	};
} ; // namespace blast