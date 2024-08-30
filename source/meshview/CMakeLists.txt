add_library(Meshview external/stb/stb_image.cpp include/meshview/meshview.hpp include/meshview/common.hpp include/meshview/meshview_imgui.hpp include/meshview/util.hpp include/meshview/internal/assert.hpp include/meshview/internal/shader.hpp include/meshview/internal/shader_inline.hpp src/camera.cpp src/mesh.cpp src/shader.cpp src/texture.cpp src/util.cpp src/viewer.cpp)
target_include_directories(Meshview PUBLIC external)
target_include_directories(Meshview PUBLIC include)
target_link_libraries(Meshview PUBLIC OpenGL::GL glfw GLEW::GLEW Eigen3::Eigen imgui::imgui)

add_executable(Meshview_example example.cpp)
target_link_libraries(Meshview_example PRIVATE Meshview)

