#include "visualizer.h"

void blast::VisualNode::draw()
{
	DrawCircleLinesV(renderPosition, Visualizer::NODE_RADIUS, renderColor);
}

void blast::Visualizer::initializeWindow()
{
	InitWindow(1280, 720, "Blast!");
	SetTargetFPS(60);
}

void blast::Visualizer::mainLoop()
{
	while (!WindowShouldClose()) {
		ClearBackground(RAYWHITE);

		BeginDrawing();
		render();
		EndDrawing();
	}
}

void blast::Visualizer::render()
{
}
