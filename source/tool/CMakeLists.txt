add_executable(Tool src/startup.cpp src/tool.hpp src/tool.cpp src/pipeline.hpp src/pipeline.cpp
	include/blast/common.hpp include/blast/meshview.hpp include/blast/meshview_imgui.hpp include/blast/util.hpp
	include/blast/internal/assert.hpp include/blast/internal/shader_inline.hpp
	src/camera.cpp src/mesh.cpp src/shader.cpp src/texture.cpp src/util.cpp src/viewer.cpp
	external/stb/stb_image.cpp)
target_link_libraries(Tool PUBLIC Core OpenGL::GL glfw GLEW::GLEW Eigen3::Eigen imgui::imgui)
target_include_directories(Tool PUBLIC external)
target_include_directories(Tool PUBLIC include)

set_target_properties(Tool PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")
add_custom_command(TARGET Tool POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/examples ${CMAKE_CURRENT_BINARY_DIR}/examples)

