#include "visualizer.h"

void blast::VisualNode::draw()
{
	DrawCircleLinesV(renderPosition, Visualizer::NODE_RADIUS, renderColor);
	for (int i = 0; i < Visualizer::EDGE_THICKNESS; i++) {
		DrawCircleLinesV(renderPosition, Visualizer::NODE_RADIUS - i, renderColor);
	}
	
	// RLAPI void DrawTextEx(Font font, const char *text, Vector2 position, float fontSize, float spacing, Color tint);
	DrawCircleV(renderPosition, Visualizer::NODE_RADIUS - Visualizer::EDGE_THICKNESS, WHITE);
	DrawTextEx(GetFontDefault(), label.c_str(), { renderPosition.x - (Visualizer::NODE_RADIUS / 2) - 7, renderPosition.y - 4 }, 14, 1, BLACK);
}

inline blast::VisualNodeP blast::Visualizer::getNode(int index)
{
	return static_pointer_cast<VisualNode>(graph->getNode(index));
}


void blast::Visualizer::initializeWindow()
{
	InitWindow(1280, 720, "Blast!");
	SetTargetFPS(60);
}

void blast::Visualizer::mainLoop()
{
	while (!WindowShouldClose()) {
		update();
		ClearBackground(RAYWHITE);

		BeginDrawing();
		render();
		EndDrawing();
	}
}

void blast::Visualizer::update()
{
	// Add node
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		Vector2 mousePosition = GetMousePosition();
		Color color{};

		VisualNodeP node = std::make_shared<VisualNode>("Node " + std::to_string(graph->getNodeCount()), mousePosition, color);
		graph->addNode(node);
	}


	if (IsKeyPressed(KEY_E)) {
		// Add edge
		if (graph->getNodeCount() >= 2) {
			int node1 = GetRandomValue(0, graph->getNodeCount() - 1);
			int node2 = GetRandomValue(0, graph->getNodeCount() - 1);

			graph->addDirectedEdge(node1, node2, 1);
		}
	}
}

void blast::Visualizer::render()
{
	for (int i = 0; i < graph->getNodeCount(); i++) {
		for (int j = 0; j < graph->getNodeCount(); j++) {
			if (graph->existsEdge(i, j)) {
				DrawLineV(getNode(i)->renderPosition, getNode(j)->renderPosition, BLACK);
			}
		}
	}

	for (int i = 0; i < graph->getNodeCount(); i++) {
		VisualNodeP node = getNode(i);
		// node->renderPosition = { 100.f + i * 100.f, 100.f };
		node->renderColor = RED;
		node->draw();
	}
}
