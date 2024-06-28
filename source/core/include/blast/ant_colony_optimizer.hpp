#pragma once

#include "graph.hpp"
#include "optimizer_strategy.hpp"

namespace blast {
	class Ant_colony_optimizer : public Optimizer_strategy
	{
	public:
		Ant_colony_optimizer(const int ant_count, const int iteration_count) : ant_count(ant_count), iteration_count(iteration_count) {}
		int ant_count;
		int iteration_count;

		std::vector<size_t> execute(const blast::Graph& node_graph);
	private:
		std::vector<std::vector<size_t>> construct_solution(const blast::Graph& pheromone_graph,
		                                                    const blast::Graph& node_graph) const;
		void global_pheromone_update(const blast::Graph& node_graph, blast::Graph& pheromone_graph, const std::vector<std::vector<size_t>>& taken_paths);
		void evaporate_pheromone(blast::Graph& pheromone_graph);
	};
} ; // namespace blast