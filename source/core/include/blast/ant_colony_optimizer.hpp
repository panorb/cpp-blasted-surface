#pragma once

#include "graph.hpp"

namespace blast {
	class AntColonyOptimizer
	{
	public:
		AntColonyOptimizer(const int ant_count, const int iteration_count) : ant_count(ant_count), iteration_count(iteration_count) {}
		int ant_count;
		int iteration_count;

		std::vector<size_t> execute(blast::Graph& node_graph);
	private:
		std::vector<size_t> construct_solution(blast::Graph& node_graph) const;
		void global_pheromone_update(const std::vector<size_t>& best_solution);
		void evaporate_pheromone();
	};
} ; // namespace blast