#pragma once

#include "graph.hpp"
#include "optimizer_strategy.hpp"
#include <Eigen/Core>

namespace blast
{
	class Three_point_optimizer : public Optimizer_strategy
	{
	private:
		const Graph* node_graph; // non-owning pointer
		const std::vector<Eigen::Vector3f> points;
	public:
		Three_point_optimizer(const Graph* node_graph, const std::vector<Eigen::Vector3f> points) : node_graph(node_graph), points(points) {};
		std::vector<size_t> execute() const;
	};
}