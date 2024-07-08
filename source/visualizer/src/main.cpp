#include <blast/visualizer.hpp>

#include "blast/log.hpp"

int main()
{
	blast::log::init();

	auto graph = std::make_shared<blast::Graph>();
	blast::Visualizer visualizer{ graph };

	visualizer.initialize_window();
	visualizer.main_loop();
	return 0;
}
