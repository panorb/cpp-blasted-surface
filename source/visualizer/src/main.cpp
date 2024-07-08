#include <blast/visualizer.hpp>
#include <spdlog/cfg/env.h>

int main()
{
	spdlog::cfg::load_env_levels();

	auto graph = std::make_shared<blast::Graph>();
	blast::Visualizer visualizer{ graph };

	visualizer.initialize_window();
	visualizer.main_loop();
	return 0;
}
