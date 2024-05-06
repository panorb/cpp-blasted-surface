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

	class Visualizer;

	class VisualizerState {
	public:
		virtual ~VisualizerState() {};
		virtual void update(Visualizer& visualizer) {}
		virtual std::string name() = 0;
	};

	class AddNodeState : public VisualizerState {
	public:
		void update(Visualizer& visualizer) override;
		std::string name() override { return "Add Node"; };
	};

	class AddEdgeState : public VisualizerState {
		int firstIndex = -1;
	public:
		void update(Visualizer& visualizer) override;
		std::string name() override { return "Add Edge"; };
	};

	class Visualizer {
		std::unique_ptr<VisualizerState> activeState = std::make_unique<AddNodeState>();

		friend class VisualNode;
	public:
		static const int NODE_RADIUS = 32;
		static const int EDGE_THICKNESS = 3;

		Visualizer(std::shared_ptr<blast::Graph> graph) : graph(graph) {};
		std::shared_ptr<blast::Graph> graph;
		void initializeWindow();
		void mainLoop();
		void update();
		void render();
		inline VisualNodeP getNode(int index);
	};
};
