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
		const int NODE_RADIUS = 16;
		const int EDGE_THICKNESS = 2;

	public:
		void initializeWindow();
		void render();
	};
};
