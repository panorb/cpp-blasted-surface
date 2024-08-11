add_library(Meshview include/meshview/meshview.hpp include/meshview/meshview_imgui.hpp include/meshview/util.hpp include/meshview/internal/assert.hpp include/meshview/internal/shader.hpp include/meshview/internal/shader_inline.hpp src/camera.cpp src/mesh.cpp src/shader.cpp src/texture.cpp src/util.cpp src/viewer.cpp)
target_include_directories(Meshview PUBLIC include)

