add_library(SegmentPlanes include/blast/boundary_volume_hierarchy.hpp include/blast/bounding_box.hpp include/blast/disjoint_set.hpp include/blast/octree_depth_indexable.hpp include/blast/oliveira_planes.hpp include/blast/planar_patch.hpp include/blast/plane_detector.hpp include/blast/segmenter_utility.hpp src/boundary_volume_hierarchy.cpp src/bounding_box.cpp src/disjoint_set.cpp src/octree_depth_indexable.cpp src/oliveira_planes.cpp src/planar_patch.cpp src/plane_detector.cpp src/segmenter_utility.cpp)

target_link_libraries(SegmentPlanes PRIVATE ${PCL_LIBRARIES} Qhull::qhullcpp)
target_include_directories(SegmentPlanes PUBLIC include)

