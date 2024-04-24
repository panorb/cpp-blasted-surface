cmake_minimum_required(VERSION 3.15)

project(BlastedSurface LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

find_package(raylib 3.0 REQUIRED)
find_package(Catch2 3 CONFIG REQUIRED)

add_subdirectory(core)
add_subdirectory(aco)
add_subdirectory(visualizer)

