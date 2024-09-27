add_executable(Playground src/playground.cpp)

target_link_libraries(Playground PRIVATE Core Meshview)
set_target_properties(Playground PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")
#add_custom_command(TARGET Playground POST_BUILD
#	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/examples ${CMAKE_CURRENT_BINARY_DIR}/examples)

