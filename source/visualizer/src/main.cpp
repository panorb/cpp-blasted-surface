#include <blast/visualizer.h>

int main()
{
	auto graph = std::make_shared<blast::Graph>();
	blast::Visualizer visualizer{ graph };

	visualizer.initialize_window();
	visualizer.main_loop();
	return 0;
}
