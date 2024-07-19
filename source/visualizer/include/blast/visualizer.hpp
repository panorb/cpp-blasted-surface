#pragma once
#include <raylib.h>
#include <raymath.h>
#include <blast/node.hpp>
#include <blast/graph.hpp>

#include "blast/ant_colony_optimizer.hpp"

namespace blast {
	class Visual_node : public Node {
	public:
		Vector2 render_position{};
		Color render_color{};

		explicit Visual_node(std::string label) : Node(label) {};
		Visual_node(std::string label, Vector2 position, Color color) : render_position(position), render_color(color), Node(label) {};
		void draw();
	};

	class Visualizer;

	class Visualizer_state {
	public:
		virtual std::string name() = 0;
		virtual ~Visualizer_state() {};
		virtual void enter(Visualizer& visualizer) {};
		virtual void update(Visualizer& visualizer) {};
		virtual void draw(Visualizer& visualizer) {};
		virtual void exit(Visualizer& visualizer) {};
	};

	class Add_node_state final : public Visualizer_state {
	public:
		std::string name() override { return "Add Node"; };
		void update(Visualizer& visualizer) override;
	};

	class Add_edge_state final : public Visualizer_state {
		int first_index = -1;
	public:
		std::string name() override { return "Add Edge"; };
		void update(Visualizer& visualizer) override;
		void draw(Visualizer& visualizer) override;
	};

	class Move_state final : public Visualizer_state {
		int held_index = -1;
	public:
		std::string name() override { return "Move Node"; };
		void exit(Visualizer& visualizer) override;
		void update(Visualizer& visualizer) override;
	};

	class Run_aco_state final : public Visualizer_state {
		std::unique_ptr<Ant_colony_optimizer> aco;
		const float TIME_PER_STEP = 3.0f;
		float time_delta = 0.0f;
	public:
		Run_aco_state() = delete;
		explicit Run_aco_state(std::unique_ptr<Ant_colony_optimizer> aco): aco(std::move(aco)) {}
		std::string name() override { return "Run ACO"; };
		void update(Visualizer& visualizer) override;
	};

	class Visualizer {
		std::unique_ptr<Visualizer_state> active_state = std::make_unique<Add_node_state>();

		friend class Visual_node;
	public:
		static constexpr int node_radius = 12;
		static constexpr int edge_thickness = 3;
		static constexpr auto default_node_color = BLACK;

		Visualizer(std::shared_ptr<blast::Graph> graph) : graph(graph) {};
		std::shared_ptr<blast::Graph> graph;
		void change_state(std::unique_ptr<Visualizer_state> state);
		void initialize_window();
		void main_loop();
		void update();
		void render();
		int get_hovered_node_index(Vector2 cursor_pos);
		inline std::shared_ptr<Visual_node> get_node(size_t index) const;
	};
}; // namespace blast
