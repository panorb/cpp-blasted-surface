#pragma once

#include "blast/common.hpp"

#ifdef MESHVIEW_IMGUI
#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>
#include <imgui_stdlib.h>
#endif
