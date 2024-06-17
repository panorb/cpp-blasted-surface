add_executable(Visualizer include/blast/visualizer.hpp src/visualizer.cpp src/main.cpp)
target_include_directories(Visualizer PUBLIC include)
target_link_libraries(Visualizer PRIVATE Core raylib)

