#pragma once
#include <raylib.h>
#include <raymath.h>
#include <node.h>
#include <graph.h>

namespace blast {
	class VisualNode : public Node {
	public:
		Vector2 renderPosition{};
		Color renderColor{};

		VisualNode(std::string label) : Node(label) {};
		VisualNode(std::string label, Vector2 position, Color color) : renderPosition(position), renderColor(color), Node(label) {};
		void draw();
	};

	typedef std::shared_ptr<VisualNode> VisualNodeP;

	class Visualizer {
		static const int NODE_RADIUS = 32;
		static const int EDGE_THICKNESS = 3;
		inline VisualNodeP getNode(int index);

		friend class VisualNode;
	public:
		Visualizer(std::shared_ptr<blast::Graph> graph) : graph(graph) {};
		std::shared_ptr<blast::Graph> graph;
		void initializeWindow();
		void mainLoop();
		void update();
		void render();
	};
};
