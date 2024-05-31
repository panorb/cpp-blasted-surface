add_executable(Vulkanizer include/vulkanizer.h src/vulkanizer.cpp include/vulkan_renderer.h src/vulkan_renderer.cpp src/main.cpp include/vulkan_ext_functions.h src/vulkan_ext_functions.cpp shaders/shader.frag shaders/shader.vert)
target_link_libraries(Vulkanizer PRIVATE glfw glm::glm Vulkan::Vulkan)
target_include_directories(Vulkanizer PRIVATE include)
add_shaders(Vulkanizer shaders/shader.frag shaders/shader.vert)

