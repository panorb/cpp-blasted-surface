add_executable(Editor src/editor.cpp)
target_link_libraries(Editor PRIVATE Core Meshview SegmentPlanes)
set_target_properties(Editor PROPERTIES VS_DEBUGGER_ENVIRONMENT "SPDLOG_LEVEL=info")

