#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan_ext_functions.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>

#include <optional>
// Container classes
#include <vector>
#include <array>

class Vulkan_renderer {
public:
	static Vulkan_renderer* get_instance();
	bool framebuffer_resized = false;

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
	void main_loop();
private:
	const int max_frames_in_flight = 2;
	GLFWwindow* window;

	const std::vector<const char*> validation_layers = {
		"VK_LAYER_KHRONOS_validation"
	};

#ifndef NDEBUG
	const bool enable_validation_layers = true;
#else
	const bool enable_validation_layers = false;
#endif

	///////////////////////////
	// HELPERS               //
	///////////////////////////

	struct Queue_family_indices {
		std::optional<uint32_t> graphics_family;
		std::optional<uint32_t> present_family;

		bool is_complete() const {
			return graphics_family.has_value() && present_family.has_value();
		}
	};

	struct Swap_chain_support_details {
		VkSurfaceCapabilitiesKHR capabilities;
		std::vector<VkSurfaceFormatKHR> formats;
		std::vector<VkPresentModeKHR> present_modes;
	};

	struct Vertex {
		glm::vec2 pos;
		glm::vec3 color;

		static VkVertexInputBindingDescription get_binding_description() {
			VkVertexInputBindingDescription binding_description = {};
			binding_description.binding = 0;
			binding_description.stride = sizeof(Vertex);
			binding_description.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

			return binding_description;
		}

		static std::array<VkVertexInputAttributeDescription, 2> get_attribute_descriptions() {
			std::array<VkVertexInputAttributeDescription, 2> attribute_descriptions = {};

			// Position
			attribute_descriptions[0].binding = 0;
			attribute_descriptions[0].location = 0;
			attribute_descriptions[0].format = VK_FORMAT_R32G32_SFLOAT;
			attribute_descriptions[0].offset = offsetof(Vertex, pos);

			// Color
			attribute_descriptions[1].binding = 0;
			attribute_descriptions[1].location = 1;
			attribute_descriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
			attribute_descriptions[1].offset = offsetof(Vertex, color);

			return attribute_descriptions;
		}
	};

	struct Uniform_buffer_object
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
	VkDebugUtilsMessengerEXT debug_messenger = VK_NULL_HANDLE;
	VkPhysicalDevice physical_device = VK_NULL_HANDLE;
	VkDevice device = VK_NULL_HANDLE;
	
	VkQueue graphics_queue = VK_NULL_HANDLE;

	VkSurfaceKHR surface = VK_NULL_HANDLE;
	VkQueue present_queue = VK_NULL_HANDLE;

	VkSwapchainKHR swap_chain = VK_NULL_HANDLE;
	std::vector<VkImage> swap_chain_images;
	VkFormat swap_chain_image_format = {};
	VkExtent2D swap_chain_extent = {};
	std::vector<VkImageView> swap_chain_image_views;
	std::vector<VkFramebuffer> swap_chain_framebuffers;

	VkDescriptorSetLayout descriptor_set_layout = VK_NULL_HANDLE;
	VkDescriptorPool descriptor_pool = VK_NULL_HANDLE;
	std::vector<VkDescriptorSet> descriptor_sets;
	
	VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
	VkRenderPass render_pass = VK_NULL_HANDLE;
	VkPipeline graphics_pipeline = VK_NULL_HANDLE;

	VkBuffer vertex_buffer = VK_NULL_HANDLE;
	VkDeviceMemory vertex_buffer_memory = VK_NULL_HANDLE;

	std::vector<VkBuffer> uniform_buffers;
	std::vector<VkDeviceMemory> uniform_buffers_memory;
	std::vector<void*> uniform_buffers_mapped;

	VkCommandPool command_pool = VK_NULL_HANDLE;
	std::vector<VkCommandBuffer> command_buffers;

	std::vector<VkSemaphore> image_available_semaphores;
	std::vector<VkSemaphore> render_finished_semaphores;
	std::vector<VkFence> in_flight_fences;
	uint32_t current_frame = 0;

	/**
	* @brief Initializes the GLFW window
	*/
	void init_window();
	/**
	* @brief Initializes the Vulkan API
	*/
	void init_vulkan();
	/**
	* @brief Setting up the debug messenger
	*/
	void setup_debug_messenger();
	/**
	* @brief Checks if the validation layers are supported
	* @return true if the validation layers are supported, false otherwise
	*/
	bool check_validation_layer_support() const;
	/**
	* @brief Populates the debug messenger create info
	*/
	void populate_debug_messenger_create_info(VkDebugUtilsMessengerCreateInfoEXT& create_info) const;
	
	/**
	* @brief Creates the Vulkan instance
	*/
	void create_instance();
	/**
	* @brief Returns the required extensions for the Vulkan instance based on whether validation layers are enabled or not
	*/
	std::vector<const char*> get_required_instance_extensions() const;
	/**
	* @brief Creates the window surface to render to
	*/
	void create_surface();

	/**
	* @brief Selects the first physical device that supports the required features
	*/
	void pick_physical_device();
	/**
	* @brief Checks if the physical device supports the required features
	*/
	bool is_device_suitable(VkPhysicalDevice device) const;
	/**
	* @brief Returns the queue families supported by the physical device
	*/
	Queue_family_indices find_queue_families(VkPhysicalDevice device) const;
	/**
	* @brief Returns the required device extensions
	*/
	std::vector<const char*> get_required_device_extensions() const;
	/**
	* @brief Checks if the physical device supports the required extensions
	*/
	bool check_device_extension_support(VkPhysicalDevice device) const;

	/**
	* @brief Creates the logical device
	*/
	void create_logical_device();

	/**
	* @brief Creates the swap chain
	*/
	void create_swap_chain();
	/**
	* @brief Queries the swap chain support details for the physical device
	*/
	Swap_chain_support_details query_swap_chain_support(VkPhysicalDevice device) const;
	/**
	* @brief Chooses the swap surface format
	*/
	VkSurfaceFormatKHR choose_swap_surface_format(const std::vector<VkSurfaceFormatKHR>& available_formats) const;
	/**
	* @brief Chooses the swap present mode
	*/
	VkPresentModeKHR choose_swap_present_mode(const std::vector<VkPresentModeKHR>& available_present_modes) const;
	/**
	* @brief Chooses the swap extent
	*/
	VkExtent2D choose_swap_extent(const VkSurfaceCapabilitiesKHR& capabilities) const;
	/**
	* @brief Creates the image views for the swap chain images
	*/
	void create_image_views();
	/**
	* @brief Creates the framebuffers
	*/
	void create_framebuffers();
	/**
	* @brief Recreates the swap chain
	*/
	void recreate_swap_chain();
	/**
	* @brief Destroys the swap chain
	*/
	void cleanup_swap_chain();

	/**
	* @brief Creates a shader module from the shader code
	*/
	VkShaderModule create_shader_module(const std::vector<char>& code) const;
	/**
	 * @brief Creates the descriptor set layout
	 */
	void create_descriptor_set_layout();
	/**
	 * @brief Creates the descriptor pool
	 */
	void create_descriptor_pool();
	/**
	 * @brief Creates the descriptor sets
	 */
	void create_descriptor_sets();
	/**
	* @brief Creates the graphics pipeline
	*/
	void create_graphics_pipeline();
	/**
	* @brief Creates the pipeline layout
	*/
	void create_render_pass();

	/**
	* @brief Creates the command pool
	*/
	void create_command_pool();
	/**
	* @brief Creates the command buffer
	*/
	void create_command_buffers();
	/**
	* @brief Records the command buffer
	*/
	void record_command_buffer(VkCommandBuffer command_buffer, uint32_t frameIndex) const;

	/**
	* @brief Creates the semaphores and fences
	*/
	void create_sync_objects();

	/**
	* @brief Draws a frame
	*/
	void draw_frame();

	/**
	* @brief Find the memory type with the specified properties
	* @param type_filter The memory type filter
	* @param properties The memory properties
	*/
	uint32_t find_memory_type(uint32_t type_filter, VkMemoryPropertyFlags properties) const;

	/**
	* @brief Creates a buffer
	* @param size The size of the buffer
	* @param usage The usage of the buffer
	* @param properties The memory properties of the buffer
	* @param buffer The buffer handle
	* @param buffer_memory The buffer memory handle
	*/
	void create_buffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& buffer_memory) const;

	/**
	* @brief Copies the buffer
	* @param src_buffer The source buffer
	* @param dst_buffer The destination buffer
	* @param size The size of the buffer
	*/
	void copy_buffer(VkBuffer src_buffer, VkBuffer dst_buffer, VkDeviceSize size) const;

	/**
	* @brief Creates the vertex buffer
	*/
	void create_vertex_buffer();

	/**
	 * @brief Creates the uniform buffers
	 */
	void create_uniform_buffers();

	/**
	 * @brief Updates the uniform buffer
	 * @param current_image The current image index
	 */
	void update_uniform_buffer(uint32_t current_image) const;

	

};
