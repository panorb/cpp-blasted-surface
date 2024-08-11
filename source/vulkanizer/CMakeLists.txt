add_executable(Vulkanizer src/VkBootstrapDispatch.h src/VkBootstrap.h src/VkBootstrap.cpp src/main.cpp include/vk_types.hpp include/vk_engine.hpp src/vk_engine.cpp include/vk_initializers.hpp src/vk_initializers.cpp include/vk_images.hpp src/vk_images.cpp include/vk_descriptors.hpp src/vk_descriptors.cpp include/vk_pipelines.hpp src/vk_pipelines.cpp shaders/gradient.comp shaders/shader.frag shaders/shader.vert)
target_link_libraries(Vulkanizer PRIVATE glfw glm::glm Vulkan::Vulkan GPUOpen::VulkanMemoryAllocator Core imgui::imgui)
target_include_directories(Vulkanizer PRIVATE include)
add_shaders(Vulkanizer shaders/gradient.comp shaders/shader.frag shaders/shader.vert)

