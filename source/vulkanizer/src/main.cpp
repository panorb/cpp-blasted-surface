#include <vk_renderer.hpp>

#include <spdlog/cfg/env.h>

int main() {
	spdlog::cfg::load_env_levels();

	Vulkan_renderer* renderer = Vulkan_renderer::get_instance();
	
	// renderer->init();
	renderer->main_loop();
	// renderer->cleanup();

	return 0;
}
