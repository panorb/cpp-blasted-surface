add_executable(Vulkanizer include/vulkanizer.hpp src/vulkanizer.cpp include/vk_renderer.hpp src/vk_renderer.cpp src/main.cpp include/vk_ext_functions.hpp src/vk_ext_functions.cpp include/vk_init.hpp src/vk_init.cpp include/vk_types.hpp src/vk_types.cpp shaders/shader.frag shaders/shader.vert)
target_link_libraries(Vulkanizer PRIVATE glfw glm::glm Vulkan::Vulkan GPUOpen::VulkanMemoryAllocator Core)
target_include_directories(Vulkanizer PRIVATE include)
add_shaders(Vulkanizer shaders/shader.frag shaders/shader.vert)

