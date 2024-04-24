add_executable(Visualizer include/visualizer.h src/visualizer.cpp)
target_include_directories(Visualizer PRIVATE include)
target_link_libraries(Visualizer PRIVATE raylib)

