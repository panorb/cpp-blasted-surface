#include "blast/ordered_optimizer.hpp"

std::vector<size_t> blast::Ordered_optimizer::execute() const
{
	// Initialize the solution vector
	std::vector<size_t> solution;

	for (size_t i = 0; i < node_graph->get_node_count(); ++i)
	{
		solution.push_back(i);
	}

	return solution;
}
