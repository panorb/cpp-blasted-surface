cmake_minimum_required(VERSION 3.15)

project(BlastedSurface LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(raylib CONFIG REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)
find_package(Vulkan REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

include(add_shaders.cmake)

add_subdirectory(core)
add_subdirectory(aco)
add_subdirectory(visualizer)
add_subdirectory(vulkanizer)

