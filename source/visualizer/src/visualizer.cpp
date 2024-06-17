#include <blast/visualizer.hpp>

void blast::Visual_node::draw()
{
	DrawCircleV(render_position, Visualizer::node_radius, render_color);
	DrawTextEx(GetFontDefault(), label.c_str(), {
		           render_position.x - (Visualizer::node_radius / 2) - 7,
		           render_position.y - Visualizer::node_radius - 14
	           }, 14, 1, BLACK);
}

inline std::shared_ptr<blast::Visual_node> blast::Visualizer::get_node(int index)
{
	return static_pointer_cast<Visual_node>(graph->get_node(index));
}


void blast::Visualizer::change_state(std::unique_ptr<Visualizer_state> state)
{
	active_state->exit(*this);
	active_state = std::move(state);
	active_state->enter(*this);
}

void blast::Visualizer::initialize_window()
{
	InitWindow(1280, 720, "Blast!");
	SetTargetFPS(60);
}

void blast::Visualizer::main_loop()
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
		change_state(std::make_unique<Add_edge_state>());
	}
	else if (IsKeyPressed(KEY_N)) {
		change_state(std::make_unique<Add_node_state>());
	}
	else if (IsKeyPressed(KEY_M)) {
		change_state(std::make_unique<Move_state>());
	}

	active_state->update(*this);
}

inline Vector2 PerpendicularCounterClockwise(Vector2 vector2)
{
	return Vector2{ -vector2.y, -vector2.x };
}

void blast::Visualizer::render()
{
	DrawText(active_state->name().c_str(), 10, 10, 20, BLACK);

	active_state->draw(*this);

	for (int i = 0; i < graph->get_node_count(); i++) {
		for (int j = 0; j < graph->get_node_count(); j++) {
			if (i < j && graph->exists_edge(i, j)) {
				DrawLineEx(get_node(i)->render_position, get_node(j)->render_position, 3.0, BLACK);
				Vector2 direction = { get_node(j)->render_position.x - get_node(i)->render_position.x, get_node(j)->render_position.y - get_node(i)->render_position.y };
				direction = Vector2Normalize(direction);
				Vector2 perpendicular = PerpendicularCounterClockwise(direction);
				perpendicular = Vector2Normalize(perpendicular);
				Vector2 middle = { (get_node(i)->render_position.x + get_node(j)->render_position.x) / 2, (get_node(i)->render_position.y + get_node(j)->render_position.y) / 2 };
				Vector2 text_position = { static_cast<float>(middle.x + (perpendicular.x * 24.0)), static_cast<float>(middle.y + (perpendicular.y * 12.0)) };
				DrawTextEx(GetFontDefault(), std::to_string(graph->get_edge_weight(i, j)).c_str(), text_position, 14, 1, BLACK);
			}
		}
	}

	for (int i = 0; i < graph->get_node_count(); i++) {
		std::shared_ptr<Visual_node> node = get_node(i);
		node->render_color = default_node_color;
		node->draw();
	}
}

int blast::Visualizer::get_hovered_node_index(Vector2 cursor_pos)
{
	int highest_index = -1;

	for (int i = 0; i < graph->get_node_count(); i++) {
		if (CheckCollisionPointCircle(cursor_pos, get_node(i)->render_position, node_radius)) {
			highest_index = i;
		}
	}

	return highest_index;
}

void blast::Add_node_state::update(Visualizer& visualizer)
{
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		Vector2 mouse_position = GetMousePosition();
		Color color{};

		std::shared_ptr<Visual_node> node = std::make_shared<Visual_node>("Node " + std::to_string(visualizer.graph->get_node_count()), mouse_position, color);
		visualizer.graph->add_node(node);
	}
	else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
		Vector2 mouse_position = GetMousePosition();
		int clicked_index = visualizer.get_hovered_node_index(mouse_position);

		if (clicked_index >= 0) {
			visualizer.graph->remove_node(clicked_index);
		}
	}
}

void blast::Add_edge_state::update(Visualizer& visualizer)
{
	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		Vector2 mouse_position = GetMousePosition();
		int clicked_index = visualizer.get_hovered_node_index(mouse_position);

		if (clicked_index >= 0) {
			if (first_index >= 0) {
				if (visualizer.graph->exists_edge(first_index, clicked_index)) {
					visualizer.graph->remove_undirected_edge(first_index, clicked_index);
				}
				else {
					visualizer.graph->add_undirected_edge(first_index, clicked_index, 1);
				}
				
				first_index = -1;
			}
			else {
				first_index = clicked_index;
			}
		}
	}
}

void blast::Add_edge_state::draw(Visualizer& visualizer) {
	if (first_index >= 0) {
		Vector2 start_position = visualizer.get_node(first_index)->render_position;
		DrawLineEx(start_position, GetMousePosition(), 3.0, RED);
	}
}

void blast::Move_state::exit(Visualizer& visualizer)
{
	SetMouseCursor(MOUSE_CURSOR_DEFAULT);
}

void blast::Move_state::update(Visualizer& visualizer)
{
	Vector2 mouse_position = GetMousePosition();
	int hovered_index = visualizer.get_hovered_node_index(mouse_position);

	if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
		held_index = hovered_index;
	}
	else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
		held_index = -1;
	}

	if (held_index >= 0) {
		DrawCircleV(mouse_position, Visualizer::node_radius + 8, RED);
		visualizer.get_node(held_index)->render_position = mouse_position;
	}
	
	if (hovered_index >= 0) {
		SetMouseCursor(MOUSE_CURSOR_RESIZE_ALL);
	}
	else {
		SetMouseCursor(MOUSE_CURSOR_DEFAULT);
	}
}
