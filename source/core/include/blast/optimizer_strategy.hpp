#pragma once

#include "graph.hpp"

namespace blast
{
	class Optimizer_strategy
	{
	public:
		virtual ~Optimizer_strategy() = default;
		virtual std::vector<size_t> execute(const blast::Graph& node_graph) = 0;
	};
}
