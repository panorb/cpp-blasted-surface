add_executable(Vulkanizer include/vulkanizer.h src/vulkanizer.cpp include/vulkan_renderer.h src/vulkan_renderer.cpp src/main.cpp)
target_link_libraries(Vulkanizer PRIVATE glfw glm::glm Vulkan::Vulkan)
target_include_directories(Vulkanizer PRIVATE include)

