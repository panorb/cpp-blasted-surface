#pragma once

#include <inttypes.h>
#include <vector>
#include <vulkan/vulkan_core.h>
#include <vk_types.hpp>
#include <vk_descriptors.hpp>

struct FrameData
{
	VkSemaphore _swapchainSemaphore, _renderSemaphore;
	VkFence _renderFence;

	VkCommandPool _commandPool;
	VkCommandBuffer _mainCommandBuffer;

	DeletionQueue _deletionQueue;
};

constexpr unsigned int FRAME_OVERLAP = 2;

class VulkanEngine {
public:
	VmaAllocator _allocator;
	DescriptorAllocator globalDescriptorAllocator;

	VkDescriptorSet _drawImageDescriptors;
	VkDescriptorSetLayout _drawImageDescriptorLayout;

	FrameData _frames[FRAME_OVERLAP];
	FrameData& get_current_frame();

	VkQueue _graphicsQueue;
	uint32_t _graphicsQueueFamily;

	bool _isInitialized{ false };
	int _frameNumber {0};
	bool stop_rendering{ false };
	VkExtent2D _windowExtent{ 1700 , 900 };

	struct GLFWwindow* _window{ nullptr };

	VkInstance _instance;
	VkDebugUtilsMessengerEXT _debug_messenger;
	VkPhysicalDevice _chosenGPU;
	VkDevice _device;
	VkSurfaceKHR _surface;

	VkSwapchainKHR _swapchain;
	VkFormat _swapchainImageFormat;

	std::vector<VkImage> _swapchainImages;
	std::vector<VkImageView> _swapchainImageViews;
	VkExtent2D _swapchainExtent;
	DeletionQueue _mainDeletionQueue;

	//draw resources
	AllocatedImage _drawImage;
	VkExtent2D _drawExtent;
	VkPipeline _gradientPipeline;
	VkPipelineLayout _gradientPipelineLayout;

	// immediate submit structures
	VkFence _immFence;
	VkCommandBuffer _immCommandBuffer;
	VkCommandPool _immCommandPool;

	static VulkanEngine& Get();

	//initializes everything in the engine
	void init();

	//shuts down the engine
	void cleanup();

	void draw_background(VkCommandBuffer cmd);

	//draw loop
	void draw();

	//run main loop
	void run();

	void immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function);

private:
	void init_vulkan();
	void create_swapchain(uint32_t width, uint32_t height);
	void init_swapchain();
	void destroy_swapchain();
	void init_commands();
	void init_sync_structures();
	void init_descriptors();

	void init_pipelines();
	void init_background_pipelines();
	void init_imgui();
	void draw_imgui(VkCommandBuffer cmd, VkImageView targetImageView);
};
