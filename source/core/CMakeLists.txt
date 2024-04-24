add_library(Core include/graph.h include/node.h src/graph.cpp src/node.cpp)
target_include_directories(Core PUBLIC include)

add_executable(CoreTest tests/graph.test.cpp)
target_link_libraries(CoreTest PRIVATE Core Catch2::Catch2WithMain)

