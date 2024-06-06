add_library(Core include/blast/graph.h src/graph.cpp include/blast/node.h src/node.cpp include/blast/point_cloud.h src/point_cloud.cpp)
target_include_directories(Core PUBLIC include)
target_link_libraries(Core PUBLIC glm::glm)
target_link_libraries(Core PRIVATE ${PCL_LIBRARIES})

add_executable(CoreTest tests/graph.test.cpp tests/point_cloud.test.cpp)
target_link_libraries(CoreTest PRIVATE Core Catch2::Catch2WithMain)

