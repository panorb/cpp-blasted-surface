#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <stdexcept>
#include <optional>
// Container classes
#include <vector>
#include <set>
#include <fstream>

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

	/**
	* @brief Initializes the Vulkan renderer
	*/
	void init();
	/**
	* @brief Cleans up the Vulkan renderer
	*/
	void cleanup();
private:
	GLFWwindow* window;

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

	/**
	* @brief Initializes the GLFW window
	*/
	void initWindow();
	/**
	* @brief Initializes the Vulkan API
	*/
	void initVulkan();
	
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


};
