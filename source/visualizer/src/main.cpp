#include <visualizer.h>

int main()
{
	std::shared_ptr<blast::Graph> graph = std::make_shared<blast::Graph>();
	blast::Visualizer visualizer{ graph };

	visualizer.initializeWindow();
	visualizer.mainLoop();
	return 0;
}
