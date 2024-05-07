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
	if (IsKeyPressed(KEY_E)) {
		activeState = std::make_unique<AddEdgeState>();
	}
	else if (IsKeyPressed(KEY_N)) {
		activeState = std::make_unique<AddNodeState>();
	}

	activeState->update(*this);
}

void blast::Visualizer::render()
{
	DrawText(activeState->name().c_str(), 10, 10, 20, BLACK);

	activeState->draw(*this);

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

void blast::AddNodeState::update(Visualizer& visualizer)
{
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		Vector2 mousePosition = GetMousePosition();
		Color color{};

		VisualNodeP node = std::make_shared<VisualNode>("Node " + std::to_string(visualizer.graph->getNodeCount()), mousePosition, color);
		visualizer.graph->addNode(node);
	}
}

void blast::AddEdgeState::update(Visualizer& visualizer)
{
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		Vector2 mousePosition = GetMousePosition();
		int highestIndex = -1;

		for (int i = 0; i < visualizer.graph->getNodeCount(); i++) {
			if (CheckCollisionPointCircle(mousePosition, visualizer.getNode(i)->renderPosition, Visualizer::NODE_RADIUS)) {
				highestIndex = i;
			}
		}

		if (highestIndex >= 0) {
			if (firstIndex >= 0) {
				visualizer.graph->addUndirectedEdge(firstIndex, highestIndex, 1);
				firstIndex = -1;
			}
			else {
				firstIndex = highestIndex;
			}
		}
	}
}

void blast::AddEdgeState::draw(Visualizer& visualizer) {
	if (firstIndex >= 0) {
		Vector2 startPosition = visualizer.getNode(firstIndex)->renderPosition;
		DrawLineV(startPosition, GetMousePosition(), RED);
	}
}
