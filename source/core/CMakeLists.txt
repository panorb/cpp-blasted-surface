set(plane_segmenter_headers include/blast/planes/boundary_volume_hierarchy.hpp include/blast/planes/bounding_box.hpp include/blast/planes/disjoint_set.hpp include/blast/planes/octree_depth_indexable.hpp include/blast/planes/oliveira_planes.hpp include/blast/planes/planar_patch.hpp include/blast/planes/plane_detector.hpp include/blast/planes/segmenter.hpp include/blast/planes/segmenter_utility.hpp include/blast/sampler/grid_sampler.hpp)
set(plane_segmenter_source src/planes/boundary_volume_hierarchy.cpp src/planes/bounding_box.cpp src/planes/disjoint_set.cpp src/planes/octree_depth_indexable.cpp src/planes/oliveira_planes.cpp src/planes/planar_patch.cpp src/planes/plane_detector.cpp src/planes/segmenter_utility.cpp src/sampler/grid_sampler.cpp)

add_library(Core include/blast/graph.hpp src/graph.cpp include/blast/node.hpp src/node.cpp include/blast/point_cloud.hpp src/point_cloud.cpp include/blast/optimizer_strategy.hpp include/blast/ant_colony_optimizer.hpp src/ant_colony_optimizer.cpp include/blast/greedy_optimizer.hpp src/greedy_optimizer.cpp include/blast/three_point_optimizer.hpp src/three_point_optimizer.cpp include/blast/triangulation.hpp src/triangulation.cpp include/blast/voxel_grid.hpp src/voxel_grid.cpp include/blast/utility.hpp ${plane_segmenter_headers} ${plane_segmenter_source})
target_include_directories(Core PUBLIC include)
target_link_libraries(Core PUBLIC spdlog::spdlog Eigen3::Eigen CGAL::CGAL)
target_link_libraries(Core PRIVATE ${PCL_LIBRARIES} Qhull::qhullcpp)

# Source groups
# source_group("Plane Segmentation/Header Files" FILES ${plane_segmenter_headers})
# source_group("Plane Segmentation/Source Files" FILES ${plane_segmenter_source})

add_executable(CoreTest tests/graph.test.cpp tests/point_cloud.test.cpp tests/ant_colony_optimizer.test.cpp tests/setup.cpp)
target_link_libraries(CoreTest PRIVATE Core Catch2::Catch2WithMain)
set_target_properties(CoreTest PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/tests/files DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
