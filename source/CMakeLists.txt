cmake_minimum_required(VERSION 3.15)

project(BlastedSurface LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(raylib CONFIG REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include(add_shaders.cmake)

add_subdirectory(core)
add_subdirectory(visualizer)
add_subdirectory(vulkanizer)

