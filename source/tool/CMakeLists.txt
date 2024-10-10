add_executable(Tool src/startup.cpp src/tool.hpp src/tool.cpp src/pipeline.hpp src/pipeline.cpp)
target_link_libraries(Tool PRIVATE Core Meshview)
set_target_properties(Tool PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")
add_custom_command(TARGET Tool POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/examples ${CMAKE_CURRENT_BINARY_DIR}/examples)

