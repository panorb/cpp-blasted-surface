#include <vk_renderer.hpp>

#include "blast/log.hpp"

int main() {
	blast::log::init();

	Vulkan_renderer* renderer = Vulkan_renderer::get_instance();
	
	// renderer->init();
	renderer->main_loop();
	// renderer->cleanup();

	return 0;
}
