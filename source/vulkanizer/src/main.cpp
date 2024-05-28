#include <vulkan_renderer.h>

int main() {
	VulkanRenderer* renderer = VulkanRenderer::GetInstance();
	renderer->init();
	renderer->cleanup();

	return 0;
}