#include "blast/tool.hpp"

#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Geometry>

#include "blast/util.hpp"
#include "blast/internal/shader.hpp"
// Inlined shader code
#include <filesystem>
#include <spdlog/spdlog.h>

#include "blast/detected_plane_segment.hpp"
#include "blast/internal/shader_inline.hpp"

#ifdef MESHVIEW_IMGUI
#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>
#endif
#include <blast/point_cloud.hpp>

#include "blast/planes/oliveira_planes.hpp"

namespace blast {
namespace {
// Axes data
const float AXIS_LEN = 0.5f;
const float axes_verts[] = {0.0f, 0.0f, 0.0f, AXIS_LEN, 0.0f,     0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f,     AXIS_LEN, 0.0f,
                            0.0f, 0.0f, 0.0f, 0.0f,     0.0f,     AXIS_LEN};
const float axes_rgb[] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                          0.0f, 1.0f, 0.0f, 0.2f, 0.2f, 1.0f, 0.2f, 0.2f, 1.0f};

void error_callback(int error, const char* description) {
    std::cerr << description << "\n";
}

void win_key_callback(GLFWwindow* window, int key, int scancode, int action,
                      int mods) {
    blast::Tool& viewer =
        *reinterpret_cast<blast::Tool*>(glfwGetWindowUserPointer(window));
    if (!viewer.on_key(key, (blast::input::Action)action, mods))
        return;

#ifdef MESHVIEW_IMGUI
    ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    if (ImGui::GetIO().WantCaptureKeyboard) return;
#endif

    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
            case 'Q':
                glfwSetWindowShouldClose(window, GL_TRUE);
                break;
            case 'Z':
                viewer.camera.reset_view();
                break;
            case 'O':
                viewer.camera.ortho = !viewer.camera.ortho;
                viewer.camera.update_proj();
                break;
            case 'W':
                viewer.wireframe = !viewer.wireframe;
                break;
            case 'C':
                viewer.cull_face = !viewer.cull_face;
                break;
            case 'A':
                viewer.draw_axes = !viewer.draw_axes;
                break;
            case 'M':
                if (glfwGetWindowAttrib(window, GLFW_MAXIMIZED) == GLFW_TRUE) {
                    glfwRestoreWindow(window);
                } else {
                    glfwMaximizeWindow(window);
                }
                break;
            case 'F': {
                int* backup = viewer._fullscreen_backup;
                if (viewer._fullscreen) {
                    glfwSetWindowMonitor(window, nullptr, backup[0], backup[1],
                                         backup[2], backup[3], 0);
                    viewer._fullscreen = false;
                } else {
                    glfwGetWindowPos(window, &backup[0], &backup[1]);
                    glfwGetWindowSize(window, &backup[2], &backup[3]);
                    const GLFWvidmode* mode =
                        glfwGetVideoMode(glfwGetPrimaryMonitor());
                    glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0,
                                         mode->width, mode->height, 0);
                    viewer._fullscreen = true;
                }
            } break;
            case 'H':
                std::cout <<
                    R"HELP(Meshview (c) Alex Yu 2020
left click + drag:         rotate view
shift + left click + drag: pan view
middle click + drag:       pan view (alt)
ctrl + left click + drag:  roll view
Z:                         reset view
O:                         toggle orthographic
A:                         toggle axes
W:                         toggle wireframe
C:                         toggle backface culling
M:                         toggle maximize window (may not work on some systems)
F:                         toggle fullscreen window
)HELP";
                break;
        }
    }
}

void win_mouse_button_callback(GLFWwindow* window, int button, int action,
                               int mods) {
    blast::Tool& viewer =
        *reinterpret_cast<blast::Tool*>(glfwGetWindowUserPointer(window));
    glfwGetCursorPos(window, &viewer._mouse_x, &viewer._mouse_y);

    if (action == GLFW_RELEASE) viewer._mouse_button = -1;
    if (!viewer.on_mouse_button(button, (blast::input::Action)action,
                                mods)) {
        return;
    }
#ifdef MESHVIEW_IMGUI
    ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    if (ImGui::GetIO().WantCaptureMouse) return;
#endif
    if (action == GLFW_PRESS) {
        viewer._mouse_button = button;
        viewer._mouse_mods = mods;
    }
}

void win_mouse_move_callback(GLFWwindow* window, double x, double y) {
    bool prevent_default = false;

    blast::Tool& viewer =
        *reinterpret_cast<blast::Tool*>(glfwGetWindowUserPointer(window));
    double prex = viewer._mouse_x, prey = viewer._mouse_y;
    viewer._mouse_x = x, viewer._mouse_y = y;
    if (!viewer.on_mouse_move(x, y)) {
        return;
    }
    if (viewer._mouse_button != -1) {
        if ((viewer._mouse_button == GLFW_MOUSE_BUTTON_LEFT &&
             (viewer._mouse_mods & GLFW_MOD_SHIFT)) ||
            viewer._mouse_button == GLFW_MOUSE_BUTTON_MIDDLE) {
            // Pan
            viewer.camera.pan_with_mouse((float)(x - prex), (float)(y - prey));
        } else if (viewer._mouse_button == GLFW_MOUSE_BUTTON_LEFT &&
                   (viewer._mouse_mods & GLFW_MOD_CONTROL)) {
            // Roll
            viewer.camera.roll_with_mouse((float)(x - prex), (float)(y - prey));
        } else if (viewer._mouse_button == GLFW_MOUSE_BUTTON_LEFT) {
            viewer.camera.rotate_with_mouse((float)(x - prex),
                                            (float)(y - prey));
        }
    }
}

void win_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    blast::Tool& viewer =
        *reinterpret_cast<blast::Tool*>(glfwGetWindowUserPointer(window));
    if (!viewer.on_scroll(xoffset, yoffset)) {
        return;
    }
#ifdef MESHVIEW_IMGUI
    ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    if (ImGui::GetIO().WantCaptureMouse) return;
#endif
    viewer.camera.zoom_with_mouse((float)yoffset);
}

// Window resize
void win_framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    blast::Tool& viewer =
        *reinterpret_cast<blast::Tool*>(glfwGetWindowUserPointer(window));
    viewer.camera.aspect = (float)width / (float)height;
    viewer.camera.update_proj();
    glViewport(0, 0, width, height);
}
}  // namespace

Tool::Tool() : _fullscreen(false) {
    glfwSetErrorCallback(error_callback);

    if (!glfwInit()) {
        std::cerr << "GLFW failed to initialize\n";
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    background.setZero();

    light_color_ambient.setConstant(0.2f);
    light_color_diffuse.setConstant(1.f);
    light_color_specular.setConstant(0.25f);
    light_pos << 12.f, 10.f, 20.f;
}

Tool::~Tool() { glfwTerminate(); }

void Tool::show() {
    GLFWwindow* window =
        glfwCreateWindow(_width, _height, "blast", NULL, NULL);
    if (!window) {
        glfwTerminate();
        std::cerr << "GLFW window creation failed\n";
        return;
    }
    _window = (void*)window;

    camera.aspect = (float)_width / (float)_height;
    camera.update_proj();
    camera.update_view();

    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "GLEW init failed\n";
        getchar();
        glfwTerminate();
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glDepthFunc(GL_LESS);
    if (cull_face) glEnable(GL_CULL_FACE);

#ifdef MESHVIEW_IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, false);
    char* glsl_version = NULL;
    ImGui_ImplOpenGL3_Init(glsl_version);
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    glfwSetCharCallback(window, ImGui_ImplGlfw_CharCallback);
#endif

    // Compile shaders on-the-fly
    internal::Shader shader_mesh(MESH_VERTEX_SHADER, MESH_FRAGMENT_SHADER);
    internal::Shader shader_mesh_vert_color(MESH_VERTEX_SHADER_VERT_COLOR,
                                            MESH_FRAGMENT_SHADER_VERT_COLOR);
    internal::Shader shader_pc(POINTCLOUD_VERTEX_SHADER,
                               POINTCLOUD_FRAGMENT_SHADER);

    // Construct axes object
    PointCloud axes(Eigen::template Map<const Points>{axes_verts, 6, 3},
                    Eigen::template Map<const Points>{axes_rgb, 6, 3});
    axes.draw_lines();
    axes.update(true);

    // Events
    glfwSetKeyCallback(window, win_key_callback);
    glfwSetMouseButtonCallback(window, win_mouse_button_callback);
    glfwSetCursorPosCallback(window, win_mouse_move_callback);
    glfwSetScrollCallback(window, win_scroll_callback);
    glfwSetFramebufferSizeCallback(window, win_framebuffer_size_callback);
    glfwSetWindowUserPointer(window, this);

    glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);

    glfwSetWindowTitle(window, title.c_str());

    on_open();

    // Ask to re-create the buffers + textures in meshes/pointclouds
    for (auto& mesh : meshes) mesh->update(true);
    for (auto& pc : point_clouds) pc->update(true);

    auto set_light_and_camera = [&](const internal::Shader& shader) {
        shader.set_vec3("light.ambient", light_color_ambient);
        shader.set_vec3("light.diffuse", light_color_diffuse);
        shader.set_vec3("light.specular", light_color_specular);
        shader.set_vec3(
            "light.position",
            (camera.view.inverse() * light_pos.homogeneous()).head<3>());
        shader.set_vec3("viewPos", camera.get_pos());
    };

    _looping = true;
    while (!glfwWindowShouldClose(window)) {
        glClearColor(background[0], background[1], background[2], 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPolygonMode(GL_FRONT_AND_BACK, wireframe ? GL_LINE : GL_FILL);
        if (cull_face)
            glEnable(GL_CULL_FACE);
        else
            glDisable(GL_CULL_FACE);
        axes.enable(draw_axes);

        shader_pc.use();
        axes.draw(shader_pc.id, camera);
        for (auto& pc : point_clouds) {
            pc->draw(shader_pc.id, camera);
        }

        shader_mesh.use();
        set_light_and_camera(shader_mesh);
        for (auto& mesh : meshes) {
            if (mesh->shading_type == Mesh::ShadingType::texture) {
                mesh->draw(shader_mesh.id, camera);
            }
        }

        shader_mesh_vert_color.use();
        set_light_and_camera(shader_mesh_vert_color);
        for (auto& mesh : meshes) {
            if (mesh->shading_type == Mesh::ShadingType::vertex) {
                mesh->draw(shader_mesh_vert_color.id, camera);
            }
        }

        if (on_loop()) {
            for (auto& mesh : meshes) mesh->update(true);
            for (auto& pc : point_clouds) pc->update(true);
            camera.update_proj();
            camera.update_view();
        }

#ifdef MESHVIEW_IMGUI
        glOrtho(0, _width, _height, 0, 0, 1);

        // feed inputs to dear imgui, start new frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (on_gui()) {
            for (auto& mesh : meshes) mesh->update(true);
            for (auto& pc : point_clouds) pc->update(true);
            camera.update_proj();
            camera.update_view();
        }

        // Render dear imgui into screen
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif

        glfwSwapBuffers(window);
        if (loop_wait_events) {
            glfwWaitEvents();
        } else {
            glfwPollEvents();
        }
    }
    _looping = false;

    on_close();

    for (auto& mesh : meshes) {
        mesh->free_bufs();  // Delete any existing buffers to prevent memory
                            // leak
    }

#ifdef MESHVIEW_IMGUI
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
#endif
    _window = nullptr;
    glfwDestroyWindow(window);
}

Mesh& Tool::add_cube(const std::string& tag, 
					   const Eigen::Ref<const Vector3f>& cen, float side_len,
                       const Eigen::Ref<const Vector3f>& color) {
    Mesh cube = Mesh::Cube(tag);
    cube.verts_pos() *= side_len;
    cube.verts_pos().rowwise() += cen.transpose();
    return add_mesh(std::move(cube))
        .add_texture(color[0], color[1], color[2])
        .template add_texture<Texture::TYPE_SPECULAR>(color[0], color[1],
                                                      color[2]);
}

Mesh& Tool::add_square(const std::string& tag, const Eigen::Ref<const Vector3f>& cen, float side_len,
                         const Eigen::Ref<const Vector3f>& color) {
    Mesh sqr = Mesh::Square(tag);
    sqr.verts_pos() *= side_len;
    sqr.verts_pos().rowwise() += cen.transpose();
    return add_mesh(std::move(sqr))
        .add_texture(color[0], color[1], color[2])
        .template add_texture<Texture::TYPE_SPECULAR>(color[0], color[1],
                                                      color[2]);
}

Mesh& Tool::add_sphere(const std::string& tag, const Eigen::Ref<const Vector3f>& cen, float radius,
                         const Eigen::Ref<const Vector3f>& color, int rings,
                         int sectors) {
    Mesh sph = Mesh::Sphere(tag, rings, sectors);
    sph.verts_pos() *= radius;
    sph.verts_pos().rowwise() += cen.transpose();
    return add_mesh(std::move(sph))
        .set_shininess(32.f)
        .add_texture(color[0], color[1], color[2])
        .template add_texture<Texture::TYPE_SPECULAR>(color[0], color[1],
                                                      color[2]);
}

PointCloud& Tool::add_line(const Eigen::Ref<const Vector3f>& a,
                             const Eigen::Ref<const Vector3f>& b,
                             const Eigen::Ref<const Vector3f>& color) {
    return add_point_cloud(PointCloud::Line(a, b, color));
}

void Tool::delete_all(const std::string& tag)
{
    for (auto mesh_it = meshes.begin(); mesh_it != meshes.end(); ++mesh_it)
    {
        if (mesh_it->get()->tag == tag)
        {
            mesh_it = meshes.erase(mesh_it);
        }
    }
}

    // Custom code
void Tool::on_open()
{/**/}

void Tool::on_close()
{/**/}

bool Tool::on_loop()
{
	/**/
    return false;
}

std::vector<std::string> example_files;
std::string selected_example_file = "knochen-komplett.pcd";
// std::vector<std::shared_ptr<Oriented_bounding_box>> found_planes;
std::vector<Detected_plane_segment> detected_planes;
std::unique_ptr<blast::Point_cloud> base_point_cloud = nullptr;
size_t base_point_cloud_index = 0;
Oliveira_plane_segmenter oliveira_planes;

bool Tool::on_gui()
{
	/**/

	bool redraw_meshes = false;

    ImGui::Begin("Tool");

    if (ImGui::BeginCombo("Pointcloud file", selected_example_file.c_str()))
    {
        example_files.clear();
        // Get all files in directory "/examples"
        for (const auto& entry : std::filesystem::directory_iterator(R"(.\examples)"))
        {
            auto res = std::filesystem::path(entry.path()).filename().string();
            example_files.push_back(res);

            if (ImGui::Selectable(example_files.back().c_str(), example_files.back() == selected_example_file))
            {
                selected_example_file = example_files.back();
            }
        }

        ImGui::EndCombo();
    }

    if (ImGui::Button("Load point cloud"))
    {
        if (!selected_example_file.empty())
        {
            if (base_point_cloud != nullptr)
            {
                point_clouds.erase(point_clouds.begin() + base_point_cloud_index);
            }

            base_point_cloud_index = point_clouds.size();

            if (selected_example_file.ends_with(".ply"))
            {
                base_point_cloud = blast::Point_cloud::load_ply_file("./examples/" + selected_example_file);
            }
            else if (selected_example_file.ends_with(".pcd"))
            {
                base_point_cloud = blast::Point_cloud::load_pcd_file("./examples/" + selected_example_file);
            }
            else
            {
                //spdlog::error("Unsupported file format");
                return false;
            }

            size_t point_count = base_point_cloud->get_points().size();
            Points points{ point_count, 3 };

            for (size_t i = 0; i < point_count; ++i)
            {
                points.row(i)(0) = base_point_cloud->get_points()[i][0];
                points.row(i)(1) = base_point_cloud->get_points()[i][1];
                points.row(i)(2) = base_point_cloud->get_points()[i][2];
            }

            add_point_cloud(points, 0.f, 1.0f, 1.0f);
            redraw_meshes = true;
        }

    }

    ImGui::Separator();
    ImGui::Text("Oliveira Plane Segmentation");

    oliveira_planes.render_controls();
    if (ImGui::Button("Oliveira: Extract Planes"))
    {
        std::vector<Eigen::Vector3d> points = base_point_cloud->get_points();
        std::vector<Eigen::Vector3d> normals = base_point_cloud->estimate_normals(25.f);

        oliveira_planes.from_arrays(points, normals);

        auto planes = oliveira_planes.execute();
        // Visualize planes
        for (int i = 0; i < planes.size(); ++i)
        {
            auto& plane = planes[i];
            Eigen::Vector3f normal = plane->R_ * Eigen::Vector3f(0, 0, plane->extent_(2));
            normal.normalize();

            // Add to detected planes vector
            detected_planes.push_back(Detected_plane_segment(plane, normal, {}));

            auto box_points = plane->get_box_points();
            Points vertices{ box_points.size(), 3 };

            for (size_t i = 0; i < box_points.size(); ++i)
            {
                vertices.row(i)(0) = box_points[i].x();
                vertices.row(i)(1) = box_points[i].y();
                vertices.row(i)(2) = box_points[i].z();
            }

            Triangles triangles{ 12, 3 };
            triangles << 0, 1, 2,
                0, 2, 3,
                0, 4, 5,
                0, 5, 1,
                1, 5, 6,
                1, 6, 2,
                2, 6, 7,
                2, 7, 3,
                3, 7, 4,
                3, 4, 0,
                4, 7, 6,
                4, 6, 5;


            float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            float b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
            plane->color_ = Eigen::Vector3f(r, g, b);

            add_mesh("oliveira", vertices, triangles, r, g, b);
            // viewer.add_line(plane->get_center(), plane->get_center() + (2 * normal), Eigen::Vector3f(0.0, 1.0, 0.0));

        }
    }

	ImGui::End();
    return redraw_meshes;
}

bool Tool::on_key(int key, input::Action action, int mods)
{
	/**/
	spdlog::info("Key: {}, Mods: {}", key, mods);
    if (key == GLFW_KEY_SPACE)
    {
        // Shoot raycast from mouse position
        
        // Calculate the ray origin
        Eigen::Vector3f ray_origin = camera.get_pos();

        // Calculate the ray endpoint based on the mouse coordinates
        // Assuming the mouse coordinates are in normalized device coordinates (NDC)
        // where (-1, -1) is the bottom-left corner and (1, 1) is the top-right corner of the screen
		int width, height;
		glfwGetWindowSize((GLFWwindow*)_window, &width, &height);

        float ndc_x = static_cast<float>(_mouse_x) / width * 2 - 1;
        float ndc_y = 1.0f - (static_cast<float>(_mouse_y) / height) * 2;

		spdlog::debug("ndc_x: {}, ndc_y: {}", ndc_x, ndc_y);

        /* ndc_x *= aspect_ratio;

        Eigen::Vector3f ray_endpoint(ndc_x, ndc_y, -1.0f);

        // Transform the ray endpoint from NDC to world space
        
        Eigen::Matrix4f projection_matrix = camera.proj;
        Eigen::Matrix4f view_matrix = camera.view;
        Eigen::Matrix4f inverse_projection_view_matrix = (projection_matrix * view_matrix).inverse();

		Eigen::Vector4f ray_endpoint_final = inverse_projection_view_matrix * ray_endpoint.homogeneous();
        ray_endpoint_final /= ray_endpoint_final.w();
		ray_endpoint = ray_endpoint_final.head<3>(); */

        // Calculate the ray direction by subtracting the ray origin from the ray endpoint
        Eigen::Vector3f ray_direction = camera.raycast(ndc_x, ndc_y); // (ray_endpoint - ray_origin).normalized();

		// Find the intersection point of the ray with the planes
		/* for (auto& plane : detected_planes)
		{
			if (plane.intersect_ray(ray_origin, ray_direction))
			{
				spdlog::info("Intersection detected");
				//spdlog::info("Intersection point: ({}, {}, {})", intersection_point.x(), intersection_point.y(), intersection_point.z());
				//add_line(ray_origin, intersection_point, Eigen::Vector3f(0, 1, 0));
			}
		} */

		// Visualize the ray


		add_line(ray_origin, ray_origin + ray_direction * 10000, Eigen::Vector3f(1, 0, 0));
    }

    return true;
}

bool Tool::on_mouse_button(int key, input::Action action, int mods)
{
	/**/
    return true;
}

bool Tool::on_mouse_move(double x, double y)
{
	/**/
    return true;
}

bool Tool::on_scroll(double xoffset, double yoffset)
{
	/**/
    return true;
}
}  // namespace blast
