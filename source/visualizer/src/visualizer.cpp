#include "visualizer.h"

void blast::VisualNode::draw()
{
	DrawCircleV(renderPosition, Visualizer::NODE_RADIUS, renderColor);
	DrawTextEx(GetFontDefault(), label.c_str(), { renderPosition.x - (Visualizer::NODE_RADIUS / 2) - 7, renderPosition.y - Visualizer::NODE_RADIUS - 14 }, 14, 1, BLACK);
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
				DrawLineEx(getNode(i)->renderPosition, getNode(j)->renderPosition, 3.0, BLACK);
			}
		}
	}

	for (int i = 0; i < graph->getNodeCount(); i++) {
		VisualNodeP node = getNode(i);
		node->renderColor = DEFAULT_NODE_COLOR;
		node->draw();
	}
}

int blast::Visualizer::getClickedNodeIndex(Vector2 clickedPos)
{
	int highestIndex = -1;

	for (int i = 0; i < graph->getNodeCount(); i++) {
		if (CheckCollisionPointCircle(clickedPos, getNode(i)->renderPosition, NODE_RADIUS)) {
			highestIndex = i;
		}
	}

	return highestIndex;
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
		int clickedIndex = visualizer.getClickedNodeIndex(mousePosition);

		if (clickedIndex >= 0) {
			if (firstIndex >= 0) {
				if (visualizer.graph->existsEdge(firstIndex, clickedIndex)) {
					visualizer.graph->removeUndirectedEdge(firstIndex, clickedIndex);
				}
				else {
					visualizer.graph->addUndirectedEdge(firstIndex, clickedIndex, 1);
				}
				
				firstIndex = -1;
			}
			else {
				firstIndex = clickedIndex;
			}
		}
	}
}

void blast::AddEdgeState::draw(Visualizer& visualizer) {
	if (firstIndex >= 0) {
		Vector2 startPosition = visualizer.getNode(firstIndex)->renderPosition;
		DrawLineEx(startPosition, GetMousePosition(), 3.0, RED);
	}
}

void blast::MoveState::update(Visualizer& visualizer)
{
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		if (index < 0) {
			Vector2 mousePosition = GetMousePosition();
			index = visualizer.getClickedNodeIndex(mousePosition);
		}

	}
}
