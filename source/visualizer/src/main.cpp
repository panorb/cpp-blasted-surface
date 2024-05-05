#include <visualizer.h>

int main()
{
	blast::GraphP graph = std::make_shared<blast::Graph>();
	blast::Visualizer visualizer{ graph };

	visualizer.initializeWindow();
	visualizer.mainLoop();
	return 0;
}
