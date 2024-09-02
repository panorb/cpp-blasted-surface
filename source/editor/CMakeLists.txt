add_executable(Editor src/editor.cpp)
target_link_libraries(Editor PRIVATE Core Meshview)
set_target_properties(Editor PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")
add_custom_command(TARGET Editor POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/examples ${CMAKE_CURRENT_BINARY_DIR}/examples)

