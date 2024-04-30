#pragma once
#include <raylib.h>
#include <raymath.h>
#include <node.h>

namespace blast {
	class VisualNode : public Node {
		Vector2 renderPosition {};
		Color renderColor {};
	public:
		void draw();
	};

	class Visualizer {
		static const int NODE_RADIUS = 16;
		static const int EDGE_THICKNESS = 2;
		friend class VisualNode;
	public:
		void initializeWindow();
		void mainLoop();
		void render();
	};
};
