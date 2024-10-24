#pragma once
#define _USE_MATH_DEFINES
#include "meshview/meshview.hpp"

namespace blast {
class Tool {
public:
    Tool()
    {
        viewer.on_open = std::bind(&Tool::on_close, this);
        viewer.on_close = std::bind(&Tool::on_close, this);
        viewer.on_loop = std::bind(&Tool::on_loop, this);
        viewer.on_gui = std::bind(&Tool::on_gui, this);
        viewer.on_key = std::bind(&Tool::on_key, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        viewer.on_mouse_button = std::bind(&Tool::on_mouse_button, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        viewer.on_mouse_move = std::bind(&Tool::on_mouse_move, this, std::placeholders::_1, std::placeholders::_2);
        viewer.on_scroll = std::bind(&Tool::on_scroll, this, std::placeholders::_1, std::placeholders::_2);
    };
    void show();
private:
    blast::Viewer viewer;
    // * Event callbacks
    // Called after GL cnotext init
    void on_open();
    // Called when window is about to close
    void on_close();
    // Called per iter of render loop, before on_gui
    // return true if mesh/point cloud/camera data has been updated, false
    // otherwise
    bool on_loop();
#ifdef MESHVIEW_IMGUI
    // Called per iter of render loop, after on_loop
    // only avaiable if MESHVIEW_IMGUI defined.
    // Within the function, Dear ImGui is already set up,
    // ie. ready to use ImGui::Begin etc
    bool on_gui();
#endif
    // Called on key up/down/repeat: args (key, action, mods), return false to
    // prevent default see https://www.glfw.org/docs/3.3/group__mods.html on
    // mods
    bool on_key(int key, input::Action action, int mods);
    // Called on mouse up/down/repeat: args (button, action, mods), return false
    // to prevent default see https://www.glfw.org/docs/3.3/group__mods.html on
    // mods
    bool on_mouse_button(int key, input::Action action, int mods);
    // Called on mouse move: args(x, y) return false to prevent default
    bool on_mouse_move(double x, double y);
    // Called on mouse scroll: args(xoffset, yoffset) return false to prevent
    // default
    bool on_scroll(double xoffset, double yoffset);
};

}  // namespace blast
