add_library(Core include/blast/graph.hpp src/graph.cpp include/blast/node.hpp src/node.cpp include/blast/point_cloud.hpp src/point_cloud.cpp include/blast/optimizer_strategy.hpp include/blast/ant_colony_optimizer.hpp src/ant_colony_optimizer.cpp include/blast/log.hpp)
target_include_directories(Core PUBLIC include)
target_link_libraries(Core PUBLIC glm::glm spdlog::spdlog)
target_link_libraries(Core PRIVATE ${PCL_LIBRARIES})

add_executable(CoreTest tests/graph.test.cpp tests/point_cloud.test.cpp tests/ant_colony_optimizer.test.cpp tests/setup.cpp)
target_link_libraries(CoreTest PRIVATE Core Catch2::Catch2WithMain)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
