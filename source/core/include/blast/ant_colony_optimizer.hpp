#pragma once

#include "graph.hpp"
#include "optimizer_strategy.hpp"

namespace blast {

	struct Ant_colony_optimizer_iteration_result
	{
		size_t iteration_num;
		std::vector<size_t> best_solution;
		blast::Graph pheromone_graph;
	};

	class Ant_colony_optimizer : public Optimizer_strategy
	{
	private:
		const Graph *node_graph; // non-owning pointer
		std::unique_ptr<Graph> pheromone_graph;

		int ant_count;
		float evaporation_rate;
		float alpha; // Pheromone bias
		float beta; // Edge cost bias
		std::vector<size_t> best_overall_solution{};

	public:
		Ant_colony_optimizer(const Graph *node_graph, const int ant_count, const int iteration_count, const float evaporation_rate = 0.1f, const float alpha = 0.7f, const float beta = 0.3f);
		blast::Ant_colony_optimizer_iteration_result execute_iteration();

		std::vector<size_t> execute_iterations(size_t iteration_count);

		void initialize();

	private:
		std::vector<std::vector<size_t>> construct_solutions() const;
		void global_pheromone_update(const std::vector<std::vector<size_t>>& taken_paths);
		void evaporate_pheromone();
	};
} // namespace blast
