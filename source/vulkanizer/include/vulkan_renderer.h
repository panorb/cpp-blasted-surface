#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan_ext_functions.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>

#include <stdexcept>
#include <optional>
// Container classes
#include <vector>
#include <array>
#include <set>
#include <fstream>
#include <iostream>

static std::vector<char> readFile(const std::string& filename) {
	// Open the file in binary mode
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open()) {
		throw std::runtime_error("failed to open file!");
	}

	// Get the file size
	size_t fileSize = (size_t)file.tellg();
	std::vector<char> buffer(fileSize);

	// Go to the beginning of the file and read the data
	file.seekg(0);
	file.read(buffer.data(), fileSize);

	// Close the file
	file.close();

	return buffer;
}

class VulkanRenderer {
public:
	static VulkanRenderer* GetInstance();
	bool framebufferResized = false;

	/**
	* @brief Initializes the Vulkan renderer
	*/
	void init();
	/**
	* @brief Cleans up the Vulkan renderer
	*/
	void cleanup();
	/**
	* @brief Program main loop
	*/
	void mainLoop();
private:
	const int MAX_FRAMES_IN_FLIGHT = 2;
	GLFWwindow* window;

	const std::vector<const char*> validationLayers = {
		"VK_LAYER_KHRONOS_validation"
	};

#ifndef NDEBUG
	const bool enableValidationLayers = true;
#else
	const bool enableValidationLayers = false;
#endif

	///////////////////////////
	// HELPERS               //
	///////////////////////////

	struct QueueFamilyIndices {
		std::optional<uint32_t> graphicsFamily;
		std::optional<uint32_t> presentFamily;

		bool isComplete() const {
			return graphicsFamily.has_value() && presentFamily.has_value();
		}
	};

	struct SwapChainSupportDetails {
		VkSurfaceCapabilitiesKHR capabilities;
		std::vector<VkSurfaceFormatKHR> formats;
		std::vector<VkPresentModeKHR> presentModes;
	};

	struct Vertex {
		glm::vec2 pos;
		glm::vec3 color;

		static VkVertexInputBindingDescription getBindingDescription() {
			VkVertexInputBindingDescription bindingDescription = {};
			bindingDescription.binding = 0;
			bindingDescription.stride = sizeof(Vertex);
			bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

			return bindingDescription;
		}

		static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions() {
			std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {};

			// Position
			attributeDescriptions[0].binding = 0;
			attributeDescriptions[0].location = 0;
			attributeDescriptions[0].format = VK_FORMAT_R32G32_SFLOAT;
			attributeDescriptions[0].offset = offsetof(Vertex, pos);

			// Color
			attributeDescriptions[1].binding = 0;
			attributeDescriptions[1].location = 1;
			attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
			attributeDescriptions[1].offset = offsetof(Vertex, color);

			return attributeDescriptions;
		}
	};

	struct UniformBufferObject
	{
		glm::mat4 model;
		glm::mat4 view;
		glm::mat4 proj;
	};

	const std::vector<Vertex> vertices = {
		{{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
		{{0.5f, -0.5f}, {0.0f, 1.0f, 0.0f}},
		{{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
		{{0.5f, 0.5f}, {0.0f, 0.0f, 1.0f}},
		{{-0.5f, 0.5f}, {1.0f, 1.0f, 1.0f}},
		{{-0.5f, -0.5f}, {1.0f, 0.0f, 0.0f}},
	};
	
	///////////////////////////
	// VULKAN HANDLES        //
	///////////////////////////

	VkInstance instance = VK_NULL_HANDLE;
	VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
	VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	
	VkQueue graphicsQueue = VK_NULL_HANDLE;

	VkSurfaceKHR surface = VK_NULL_HANDLE;
	VkQueue presentQueue = VK_NULL_HANDLE;

	VkSwapchainKHR swapChain = VK_NULL_HANDLE;
	std::vector<VkImage> swapChainImages;
	VkFormat swapChainImageFormat;
	VkExtent2D swapChainExtent;
	std::vector<VkImageView> swapChainImageViews;
	std::vector<VkFramebuffer> swapChainFramebuffers;

	VkDescriptorSetLayout descriptorSetLayout;
	VkDescriptorPool descriptorPool;
	std::vector<VkDescriptorSet> descriptorSets;
	
	VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
	VkRenderPass renderPass = VK_NULL_HANDLE;
	VkPipeline graphicsPipeline = VK_NULL_HANDLE;

	VkBuffer vertexBuffer;
	VkDeviceMemory vertexBufferMemory;

	std::vector<VkBuffer> uniformBuffers;
	std::vector<VkDeviceMemory> uniformBuffersMemory;
	std::vector<void*> uniformBuffersMapped;

	VkCommandPool commandPool = VK_NULL_HANDLE;
	std::vector<VkCommandBuffer> commandBuffers;

	std::vector<VkSemaphore> imageAvailableSemaphores;
	std::vector<VkSemaphore> renderFinishedSemaphores;
	std::vector<VkFence> inFlightFences;
	uint32_t currentFrame = 0;

	/**
	* @brief Initializes the GLFW window
	*/
	void initWindow();
	/**
	* @brief Initializes the Vulkan API
	*/
	void initVulkan();
	/**
	* @brief Setting up the debug messenger
	*/
	void setupDebugMessenger();
	/**
	* @brief Checks if the validation layers are supported
	* @return true if the validation layers are supported, false otherwise
	*/
	bool checkValidationLayerSupport() const;
	/**
	* @brief Populates the debug messenger create info
	*/
	void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
	
	/**
	* @brief Creates the Vulkan instance
	*/
	void createInstance();
	/**
	* @brief Returns the required extensions for the Vulkan instance based on whether validation layers are enabled or not
	*/
	const std::vector<const char*> getRequiredInstanceExtensions() const;
	/**
	* @brief Creates the window surface to render to
	*/
	void createSurface();

	/**
	* @brief Selects the first physical device that supports the required features
	*/
	void pickPhysicalDevice();
	/**
	* @brief Checks if the physical device supports the required features
	*/
	bool isDeviceSuitable(VkPhysicalDevice device) const;
	/**
	* @brief Returns the queue families supported by the physical device
	*/
	const QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) const;
	/**
	* @brief Returns the required device extensions
	*/
	const std::vector<const char*> getRequiredDeviceExtensions() const;
	/**
	* @brief Checks if the physical device supports the required extensions
	*/
	bool checkDeviceExtensionSupport(VkPhysicalDevice device) const;

	/**
	* @brief Creates the logical device
	*/
	void createLogicalDevice();

	/**
	* @brief Creates the swap chain
	*/
	void createSwapChain();
	/**
	* @brief Queries the swap chain support details for the physical device
	*/
	const SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device) const;
	/**
	* @brief Chooses the swap surface format
	*/
	VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats) const;
	/**
	* @brief Chooses the swap present mode
	*/
	VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes) const;
	/**
	* @brief Chooses the swap extent
	*/
	VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities) const;
	/**
	* @brief Creates the image views for the swap chain images
	*/
	void createImageViews();
	/**
	* @brief Creates the framebuffers
	*/
	void createFramebuffers();
	/**
	* @brief Recreates the swap chain
	*/
	void recreateSwapChain();
	/**
	* @brief Destroys the swap chain
	*/
	void cleanupSwapChain();

	/**
	* @brief Creates a shader module from the shader code
	*/
	VkShaderModule createShaderModule(const std::vector<char>& code) const;
	/**
	 * @brief Creates the descriptor set layout
	 */
	void createDescriptorSetLayout();
	/**
	 * @brief Creates the descriptor pool
	 */
	void createDescriptorPool();
	/**
	 * @brief Creates the descriptor sets
	 */
	void createDescriptorSets();
	/**
	* @brief Creates the graphics pipeline
	*/
	void createGraphicsPipeline();
	/**
	* @brief Creates the pipeline layout
	*/
	void createRenderPass();

	/**
	* @brief Creates the command pool
	*/
	void createCommandPool();
	/**
	* @brief Creates the command buffer
	*/
	void createCommandBuffers();
	/**
	* @brief Records the command buffer
	*/
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t frameIndex);

	/**
	* @brief Creates the semaphores and fences
	*/
	void createSyncObjects();

	/**
	* @brief Draws a frame
	*/
	void drawFrame();

	/**
	* @brief Find the memory type with the specified properties
	* @param typeFilter The memory type filter
	* @param properties The memory properties
	*/
	uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);

	/**
	* @brief Creates a buffer
	* @param size The size of the buffer
	* @param usage The usage of the buffer
	* @param properties The memory properties of the buffer
	* @param buffer The buffer handle
	* @param bufferMemory The buffer memory handle
	*/
	void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory);

	/**
	* @brief Copies the buffer
	* @param srcBuffer The source buffer
	* @param dstBuffer The destination buffer
	* @param size The size of the buffer
	*/
	void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);

	/**
	* @brief Creates the vertex buffer
	*/
	void createVertexBuffer();

	/**
	 * @brief Creates the uniform buffers
	 */
	void createUniformBuffers();

	/**
	 * @brief Updates the uniform buffer
	 * @param currentImage The current image index
	 */
	void updateUniformBuffer(uint32_t currentImage);

	

};
