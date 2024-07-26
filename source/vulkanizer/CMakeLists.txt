add_executable(Vulkanizer src/VkBootstrapDispatch.h src/VkBootstrap.h src/VkBootstrap.cpp src/main.cpp include/vk_types.hpp include/vk_engine.hpp src/vk_engine.cpp shaders/shader.frag shaders/shader.vert)
target_link_libraries(Vulkanizer PRIVATE glfw glm::glm Vulkan::Vulkan GPUOpen::VulkanMemoryAllocator Core)
target_include_directories(Vulkanizer PRIVATE include)
add_shaders(Vulkanizer shaders/shader.frag shaders/shader.vert)

