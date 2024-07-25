#include <blast/visualizer.hpp>
#include <spdlog/spdlog.h>
#include <xlocale>

void blast::Visual_node::draw()
{
	DrawCircleV(render_position, Visualizer::node_radius, render_color);
	DrawTextEx(GetFontDefault(), label.c_str(), {
		           render_position.x - (Visualizer::node_radius / 2) - 7,
		           render_position.y - Visualizer::node_radius - 14
	           }, 14, 1, BLACK);
}

inline std::shared_ptr<blast::Visual_node> blast::Visualizer::get_node(size_t index) const
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

void blast::Visualizer::generate_graph_from_reference_coords(std::vector<Vector2>& positions) {
	Vector2 reference_size{878.f, 600.f};
	Vector2 screen_size{static_cast<float>(GetScreenWidth()), static_cast<float>(GetScreenHeight())};

	for (int i = 0; i < positions.size(); i++) {
		Vector2 screen_position = Vector2Multiply(Vector2Divide(positions[i], reference_size), screen_size);

		std::shared_ptr<Visual_node> node = std::make_shared<Visual_node>("Node " + std::to_string(graph->get_node_count()), screen_position, default_node_color);
		graph->add_node(node);
	}

	for (int i = 0; i < positions.size(); i++) {
		for (int j = 0; j < positions.size(); j++) {
			if (i < j)
			{
				graph->add_undirected_edge(i, j, Vector2Distance(positions[i], positions[j]));
			}
		}
	}
}

void blast::Visualizer::load_example_graph(size_t num)
{
	std::vector<Vector2> positions;

	switch (num)
	{
		case KEY_ONE:
			positions = {
				Vector2{ 274.f, 238.f },
				Vector2{ 678.f, 263.f },
				Vector2{ 213.f, 271.f },
				Vector2{ 624.f, 248.f },
				Vector2{ 756.f, 487.f },
				Vector2{ 631.f, 545.f },
				Vector2{ 227.f, 333.f },
				Vector2{ 43.f, 388.f },
				Vector2{ 349.f, 37.f },
				Vector2{ 132.f, 227.f }
			};
			break;
	case KEY_TWO:
		positions = {
				Vector2{818.f,244.f},
				Vector2{619.f,187.f},
				Vector2{802.f,333.f},
				Vector2{187.f,397.f},
				Vector2{345.f,228.f},
				Vector2{657.f,130.f},
				Vector2{724.f,578.f},
				Vector2{639.f,449.f},
				Vector2{315.f,300.f},
				Vector2{603.f,458.f},
				Vector2{379.f,594.f},
				Vector2{519.f,443.f},
				Vector2{625.f,103.f},
				Vector2{248.f,249.f},
				Vector2{286.f,386.f},
				Vector2{705.f,23.f},
				Vector2{119.f,186.f},
				Vector2{85.f,184.f},
				Vector2{114.f,331.f},
				Vector2{501.f,24.f},
				Vector2{522.f,10.f},
				Vector2{373.f,60.f},
				Vector2{348.f,362.f},
				Vector2{372.f,332.f},
				Vector2{145.f,124.f},
				Vector2{150.f,156.f},
				Vector2{668.f,383.f},
				Vector2{125.f,290.f},
				Vector2{327.f,390.f},
				Vector2{634.f,107.f},
				Vector2{285.f,314.f},
				Vector2{563.f,408.f},
				Vector2{19.f,391.f},
				Vector2{498.f,357.f},
				Vector2{731.f,211.f},
				Vector2{572.f,415.f},
				Vector2{780.f,488.f},
				Vector2{632.f,85.f},
				Vector2{85.f,482.f},
				Vector2{847.f,574.f},
				Vector2{344.f,179.f},
				Vector2{318.f,96.f},
				Vector2{694.f,581.f},
				Vector2{602.f,190.f},
				Vector2{101.f,273.f},
				Vector2{44.f,336.f},
				Vector2{340.f,23.f},
				Vector2{329.f,459.f},
				Vector2{504.f,194.f},
				Vector2{332.f,71.f},
				Vector2{423.f,443.f},
				Vector2{282.f,453.f},
				Vector2{118.f,444.f},
				Vector2{29.f,417.f},
				Vector2{829.f,276.f},
				Vector2{388.f,40.f},
				Vector2{726.f,429.f},
				Vector2{45.f,204.f},
				Vector2{185.f,584.f},
				Vector2{260.f,386.f},
				Vector2{685.f,50.f},
				Vector2{288.f,245.f},
				Vector2{751.f,331.f},
				Vector2{173.f,8.f},
				Vector2{647.f,204.f},
				Vector2{277.f,236.f},
				Vector2{864.f,488.f},
				Vector2{840.f,561.f},
				Vector2{229.f,584.f},
				Vector2{58.f,189.f},
				Vector2{426.f,433.f},
				Vector2{819.f,312.f},
				Vector2{109.f,452.f},
				Vector2{228.f,380.f},
				Vector2{456.f,97.f},
				Vector2{611.f,184.f},
				Vector2{721.f,582.f},
				Vector2{377.f,354.f},
				Vector2{528.f,296.f},
				Vector2{203.f,517.f}
		};

	}

	graph->clear();
	generate_graph_from_reference_coords(positions);
}

void blast::Visualizer::main_loop()
{
	//generate_graph_from_reference_coords(TODO);
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
	else if (IsKeyPressed(KEY_A)) {
		change_state(std::make_unique<Run_aco_state>(std::make_unique<Ant_colony_optimizer>(graph.get(), 70, 100)));
	}

	std::vector<int> num_keys = { KEY_ONE, KEY_TWO, KEY_THREE, KEY_FOUR, KEY_FIVE, KEY_SIX, KEY_SEVEN, KEY_EIGHT, KEY_NINE, KEY_ZERO };
	int pressed_key = -1;

	for (int num_key : num_keys)
	{
		if (IsKeyPressed(num_key))
		{
			pressed_key = num_key;
			break;
		}
	}

	if (pressed_key >= 0)
	{
		load_example_graph(pressed_key);
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

	if (draw_edges) {
		for (int i = 0; i < graph->get_node_count(); i++) {
			for (int j = 0; j < graph->get_node_count(); j++) {
				if (i < j && graph->exists_edge(i, j)) {
					DrawLineEx(get_node(i)->render_position, get_node(j)->render_position, 1.0, BLACK);
					Vector2 direction = { get_node(j)->render_position.x - get_node(i)->render_position.x, get_node(j)->render_position.y - get_node(i)->render_position.y };
					direction = Vector2Normalize(direction);
					Vector2 perpendicular = PerpendicularCounterClockwise(direction);
					perpendicular = Vector2Normalize(perpendicular);
					Vector2 middle = { (get_node(i)->render_position.x + get_node(j)->render_position.x) / 2, (get_node(i)->render_position.y + get_node(j)->render_position.y) / 2 };
					Vector2 text_position = { static_cast<float>(middle.x + (perpendicular.x * 24.0)), static_cast<float>(middle.y + (perpendicular.y * 12.0)) };
					DrawTextEx(GetFontDefault(), std::to_string(*graph->get_edge_weight(i, j)).c_str(), text_position, 14, 1, BLACK);
				}
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

void blast::Run_aco_state::enter(Visualizer& visualizer)
{
	visualizer.draw_edges = false;
	last_result = aco->execute_iteration();
}

void blast::Run_aco_state::update(Visualizer& visualizer)
{
	time_delta += GetFrameTime();


	if (time_delta >= TIME_PER_STEP)
	{
		spdlog::info("Step");
		time_delta = 0.0f;
		last_result = aco->execute_iteration();
	}
}

void blast::Run_aco_state::draw(Visualizer& visualizer)
{
	auto graph = visualizer.graph;

	for (int i = 0; i < graph->get_node_count(); i++) {
		for (int j = 0; j < graph->get_node_count(); j++) {
			if (i < j && last_result.pheromone_graph.exists_edge(i, j)) {
				DrawLineEx(visualizer.get_node(i)->render_position, visualizer.get_node(j)->render_position, 2.0f, ColorAlpha(RED, *last_result.pheromone_graph.get_edge_weight(i, j)));
			}
		}
	}

	// current iteration
	//auto best_solution_this_iteration = last_result.best_solution_this_iteration;
	//for (size_t i = 1; i < best_solution_this_iteration.size(); i++)
	//{
	//	DrawLineEx(visualizer.get_node(best_solution_this_iteration[i - 1])->render_position, visualizer.get_node(best_solution_this_iteration[i])->render_position, 2.0, DARKGREEN);
	//}
	//DrawLineEx(visualizer.get_node(best_solution_this_iteration.back())->render_position, visualizer.get_node(best_solution_this_iteration.front())->render_position, 2.0, DARKGREEN);
	//DrawLineEx(

	auto best_solution_overall = last_result.best_solution_overall;
	for (size_t i = 1; i < best_solution_overall.size(); i++)
	{
		DrawLineEx(visualizer.get_node(best_solution_overall[i - 1])->render_position, visualizer.get_node(best_solution_overall[i])->render_position, 2.0, GREEN);
	}
	DrawLineEx(visualizer.get_node(best_solution_overall.back())->render_position, visualizer.get_node(best_solution_overall.front())->render_position, 2.0, GREEN);

}

void blast::Run_aco_state::exit(Visualizer& visualizer)
{
	visualizer.draw_edges = true;
}
