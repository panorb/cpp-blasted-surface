#include "visualizer.h"

void blast::VisualNode::draw()
{
	DrawCircleV(renderPosition, Visualizer::NODE_RADIUS, renderColor);
	DrawTextEx(GetFontDefault(), label.c_str(), { renderPosition.x - (Visualizer::NODE_RADIUS / 2) - 7, renderPosition.y - Visualizer::NODE_RADIUS - 14 }, 14, 1, BLACK);
}

inline std::shared_ptr<blast::VisualNode> blast::Visualizer::getNode(int index)
{
	return static_pointer_cast<VisualNode>(graph->getNode(index));
}


void blast::Visualizer::changeState(std::unique_ptr<VisualizerState> state)
{
	activeState->exit(*this);
	activeState = std::move(state);
	activeState->enter(*this);
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
		changeState(std::make_unique<AddEdgeState>());
	}
	else if (IsKeyPressed(KEY_N)) {
		changeState(std::make_unique<AddNodeState>());
	}
	else if (IsKeyPressed(KEY_M)) {
		changeState(std::make_unique<MoveState>());
	}

	activeState->update(*this);
}

inline Vector2 PerpendicularCounterClockwise(Vector2 vector2)
{
	return Vector2{ -vector2.y, -vector2.x };
}

void blast::Visualizer::render()
{
	DrawText(activeState->name().c_str(), 10, 10, 20, BLACK);

	activeState->draw(*this);

	for (int i = 0; i < graph->getNodeCount(); i++) {
		for (int j = 0; j < graph->getNodeCount(); j++) {
			if (i < j && graph->existsEdge(i, j)) {
				DrawLineEx(getNode(i)->renderPosition, getNode(j)->renderPosition, 3.0, BLACK);
				Vector2 direction = { getNode(j)->renderPosition.x - getNode(i)->renderPosition.x, getNode(j)->renderPosition.y - getNode(i)->renderPosition.y };
				direction = Vector2Normalize(direction);
				Vector2 perpendicular = PerpendicularCounterClockwise(direction);
				perpendicular = Vector2Normalize(perpendicular);
				Vector2 middle = { (getNode(i)->renderPosition.x + getNode(j)->renderPosition.x) / 2, (getNode(i)->renderPosition.y + getNode(j)->renderPosition.y) / 2 };
				Vector2 textPosition = { middle.x + (perpendicular.x * 24.0), middle.y + (perpendicular.y * 12.0) };
				DrawTextEx(GetFontDefault(), std::to_string(graph->getEdgeWeight(i, j)).c_str(), textPosition, 14, 1, BLACK);
			}
		}
	}

	for (int i = 0; i < graph->getNodeCount(); i++) {
		std::shared_ptr<VisualNode> node = getNode(i);
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

		std::shared_ptr<VisualNode> node = std::make_shared<VisualNode>("Node " + std::to_string(visualizer.graph->getNodeCount()), mousePosition, color);
		visualizer.graph->addNode(node);
	}
	else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
		Vector2 mousePosition = GetMousePosition();
		int clickedIndex = visualizer.getClickedNodeIndex(mousePosition);

		if (clickedIndex >= 0) {
			visualizer.graph->removeNode(clickedIndex);
		}
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

void blast::MoveState::exit(Visualizer& visualizer)
{
	SetMouseCursor(MOUSE_CURSOR_DEFAULT);
}

void blast::MoveState::update(Visualizer& visualizer)
{
	Vector2 mousePosition = GetMousePosition();
	int hoveredIndex = visualizer.getClickedNodeIndex(mousePosition);

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		heldIndex = hoveredIndex;
	}
	else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
		heldIndex = -1;
	}

	if (heldIndex >= 0) {
		DrawCircleV(mousePosition, Visualizer::NODE_RADIUS + 8, RED);
		visualizer.getNode(heldIndex)->renderPosition = mousePosition;
	}
	
	if (hoveredIndex >= 0) {
		SetMouseCursor(MOUSE_CURSOR_RESIZE_ALL);
	}
	else {
		SetMouseCursor(MOUSE_CURSOR_DEFAULT);
	}
}
