#pragma once
#include <memory>
#include <blast/graph.hpp>
#include <vulkan/vulkan.h>

namespace blast
{

	class Graph_draw
	{
		std::shared_ptr<blast::Graph> graph;

	public:
		Graph_draw(std::shared_ptr<blast::Graph> graph);
	private:
		void initPipeline();
	};
}
