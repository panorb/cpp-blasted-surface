#pragma once

#include "graph.hpp"
#include "optimizer_strategy.hpp"

namespace blast {
class Greedy_optimizer : public Optimizer_strategy
{
private:
	const Graph* node_graph; // non-owning pointer
	std::unique_ptr<Graph> pheromone_graph;
public:
	Greedy_optimizer(const Graph* node_graph) : node_graph(node_graph) {};
	std::vector<size_t> execute();
};
}