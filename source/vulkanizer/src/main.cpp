#include <vk_renderer.hpp>

int main() {
	Vulkan_renderer* renderer = Vulkan_renderer::get_instance();
	
	// renderer->init();
	renderer->main_loop();
	// renderer->cleanup();

	return 0;
}
