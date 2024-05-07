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

	class Visualizer;

	class VisualizerState {
	public:
		virtual std::string name() = 0;
		virtual ~VisualizerState() {};
		virtual void update(Visualizer& visualizer) {}
		virtual void draw(Visualizer& visualizer) {};
	};

	class AddNodeState : public VisualizerState {
	public:
		std::string name() override { return "Add Node"; };
		void update(Visualizer& visualizer) override;
	};

	class AddEdgeState : public VisualizerState {
		int firstIndex = -1;
	public:
		std::string name() override { return "Add Edge"; };
		void update(Visualizer& visualizer) override;
		void draw(Visualizer& visualizer) override;
	};

	class MoveState : public VisualizerState {
		int index = -1;
	public:
		std::string name() override { return "Move Node"; };
		void update(Visualizer& visualizer) override;
	};

	class Visualizer {
		std::unique_ptr<VisualizerState> activeState = std::make_unique<AddNodeState>();

		friend class VisualNode;
	public:
		static const int NODE_RADIUS = 12;
		static const int EDGE_THICKNESS = 3;
		static constexpr Color DEFAULT_NODE_COLOR = BLACK;

		Visualizer(std::shared_ptr<blast::Graph> graph) : graph(graph) {};
		std::shared_ptr<blast::Graph> graph;
		void initializeWindow();
		void mainLoop();
		void update();
		void render();
		int getClickedNodeIndex(Vector2 clickedPos);
		inline std::shared_ptr<VisualNode> getNode(int index);
	};
}; // namespace blast
