#include "module/Viewer/src/Viewer.hpp"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <vector>

#include "VideoRecorder.hpp"
#include "module/Viewer/src/ViewCamera.hpp"
#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/RobotRoster.hpp"

// MuJoCo's include directory carries another ImGui version's imgui.h; compiling against it while
// linking ours silently mismatches struct layouts (cmake/FetchImGui.cmake).
static_assert(IMGUI_VERSION_NUM == K1SIM_IMGUI_VERSION_NUM, "viewer picked up the wrong imgui.h");

namespace k1sim::module {

    namespace {

        using viewer::Layout;
        using viewer::Mode;

        // GLFW callbacks must be plain C function pointers, and this Reactor is only
        // ever installed once (PowerPlant::install<Viewer>() is called exactly once
        // from main.cpp), so the GL/mjv/mjr state lives here as translation-unit
        // globals rather than as Viewer members — that keeps GLFW/MuJoCo headers out
        // of Viewer.hpp (main.cpp only needs the Reactor type, matching every other
        // module in this tree).

        GLFWwindow* window = nullptr;
        bool imgui_ready   = false;

        // One on-screen view: which robot it watches and how. The window shows the
        // first viewer::view_count(layout) of these; the rest keep their settings.
        struct View {
            Mode mode = Mode::CHASE;
            int robot = 0;  // index into `roster`
            mjvCamera cam{};
            viewer::ChaseParams chase;
            viewer::ChaseCamera chaser;
            std::array<mjvGLCamera, 2> gl{};  // this frame's camera, from mjv_updateCamera
        };

        std::array<View, viewer::MAX_VIEWS> views;
        Layout layout     = Layout::SIDE_BY_SIDE;
        bool show_panel   = true;

        // Minimap: an orthographic top-down render of the field in the top-right corner, with a
        // team-coloured marker and heading arrow per robot. `minimap_extent` is the ground area
        // (x, y metres, centred on the origin) it shows; `minimap_width` its share of the window.
        bool show_minimap                    = true;
        std::array<double, 2> minimap_extent = {16., 11.};
        double minimap_width                 = 0.24;
        double last_frame = 0.;

        // The robots in the scene, their trunk bodies and head cameras (-1 if missing).
        std::vector<RobotIdentity> roster;
        std::vector<int> robot_body;
        std::vector<int> robot_head;
        std::vector<std::string> robot_label;  // "Team 125  #1  (ROS domain 0)"
        std::vector<std::string> robot_name;   // "Team 125  #1"
        // Each robot's ground pose (x, y, heading rad), cached under the sim mutex each frame.
        std::vector<std::array<double, 3>> robot_pose;

        mjvCamera video_cam;
        std::array<mjvGLCamera, 2> video_gl{};

        mjvOption opt;
        mjvPerturb pert;
        mjvScene scn;
        mjrContext con;
        bool scene_ready = false;
        std::unique_ptr<VideoRecorder> video;
        std::vector<unsigned char> video_rgb;
        NUClear::clock::time_point video_start;
        int video_frames = 0, video_limit = 0;
        std::vector<RobotIdentity> video_roster;

        // Cached from SimHandles (Trigger<SimHandles, MainThread>, set once after the
        // model loads). model/data/mutex are owned by module::Simulation.
        const mjModel* g_model              = nullptr;
        mjData* g_data                      = nullptr;
        std::mutex* g_mutex                 = nullptr;
        std::atomic<double>* g_measured_rtf = nullptr;

        // Cached from SimStateUpdate (50 Hz) — overlay text only, never used for
        // anything physics-critical, so plain atomics without the sim mutex are fine.
        std::atomic<double> g_sim_time{0.0};
        std::atomic<int> g_mode{-1};

        // Set by the Viewer constructor; lets the plain-C GLFW key callback emit a
        // NUClear message (SimResetRequest) without holding a Reactor pointer here.
        std::function<void()> g_request_reset;

        // Mouse state for the simulate-style camera + perturb controls. `drag_view` is
        // the view a drag started in: it keeps the drag even if the cursor leaves it.
        bool button_left   = false;
        bool button_middle = false;
        bool button_right  = false;
        bool ui_drag       = false;  // the press landed on the ImGui panel
        int drag_view      = 0;
        double last_x      = 0.0;
        double last_y      = 0.0;

        const char* mode_name(int mode) {
            switch (mode) {
                case booster::DAMPING: return "DAMPING";
                case booster::PREPARE: return "PREPARE";
                case booster::WALKING: return "WALKING";
                case booster::CUSTOM: return "CUSTOM";
                case booster::SOCCER: return "SOCCER";
                default: return "?";
            }
        }

        int active_views() {
            return viewer::view_count(layout);
        }

        // A broadcast-style camera looking down on the whole field.
        void field_overview(mjvCamera& cam) {
            mjv_defaultCamera(&cam);
            cam.type      = mjCAMERA_FREE;
            cam.lookat[0] = cam.lookat[1] = cam.lookat[2] = 0.;
            cam.distance                                  = 12.5;
            cam.azimuth                                   = 90.;
            cam.elevation                                 = -60.;
        }

        void set_mode(View& view, Mode mode) {
            if (mode == Mode::FREE && view.mode != Mode::FREE) {
                // Leave the free camera where the view was looking, so switching is seamless.
                view.cam.type = mjCAMERA_FREE;
            }
            if (mode == Mode::CHASE) {
                view.chaser.reset();
            }
            view.mode = mode;
        }

        // Window (screen) coordinates -> framebuffer pixels, origin bottom-left.
        void to_framebuffer(GLFWwindow* w, double x, double y, double& fx, double& fy, int& fb_w, int& fb_h) {
            int win_w = 0, win_h = 0;
            glfwGetWindowSize(w, &win_w, &win_h);
            glfwGetFramebufferSize(w, &fb_w, &fb_h);
            const double sx = win_w > 0 ? double(fb_w) / win_w : 1.;
            const double sy = win_h > 0 ? double(fb_h) / win_h : 1.;
            fx              = x * sx;
            fy              = fb_h - y * sy;
        }

        int view_under(GLFWwindow* w, double x, double y) {
            double fx = 0., fy = 0.;
            int fb_w = 0, fb_h = 0;
            to_framebuffer(w, x, y, fx, fy, fb_w, fb_h);
            const int view = viewer::view_at(layout, fb_w, fb_h, fx, fy);
            return view < 0 ? 0 : view;
        }

        // mjv_select / mjv_*Perturb / mjv_moveCamera read the scene's camera, which after a
        // frame holds whichever view rendered last; point it at the view being used.
        void use_camera_of(int view) {
            scn.camera[0] = views[view].gl[0];
            scn.camera[1] = views[view].gl[1];
        }

        bool imgui_wants_mouse() {
            return imgui_ready && ImGui::GetIO().WantCaptureMouse;
        }

        // Release any perturbation force/selection so a stray drag never leaves the
        // robot pinned once the mouse button comes back up.
        void end_perturb() {
            pert.active = 0;
        }

        void keyboard_cb(GLFWwindow* w, int key, int /*scancode*/, int act, int /*mods*/) {
            if (act != GLFW_PRESS || (imgui_ready && ImGui::GetIO().WantCaptureKeyboard)) {
                return;
            }
            if (key == GLFW_KEY_ESCAPE) {
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            }
            // BACKSPACE (MuJoCo simulate convention): reset the sim to its startup state.
            if (key == GLFW_KEY_BACKSPACE && g_request_reset) {
                end_perturb();  // a reset mid-drag must not leave a perturb force pinned
                g_request_reset();
            }
            // F: shove the robot over (deterministic fall for testing FallRecovery/GetUp —
            // mouse-drag perturbs are usually within what the push-randomised policy rides out).
            if (key == GLFW_KEY_F && g_model != nullptr && g_data != nullptr && g_mutex != nullptr) {
                std::lock_guard<std::mutex> lock(*g_mutex);
                if (g_model->njnt > 0 && g_model->jnt_type[0] == mjJNT_FREE) {
                    const int dof = g_model->jnt_dofadr[0];
                    g_data->qvel[dof + 0] += 1.5;  // linear kick, world x
                    g_data->qvel[dof + 4] += 6.0;  // pitch rate — guarantees a topple
                }
            }
            // Camera keys act on the view under the cursor.
            double x = 0., y = 0.;
            glfwGetCursorPos(w, &x, &y);
            View& view = views[view_under(w, x, y)];
            if (key == GLFW_KEY_C) {
                set_mode(view, viewer::ALL_MODES[(int(view.mode) + 1) % viewer::ALL_MODES.size()]);
            }
            if (key == GLFW_KEY_V) {
                layout = viewer::ALL_LAYOUTS[(int(layout) + 1) % viewer::ALL_LAYOUTS.size()];
            }
            if ((key == GLFW_KEY_LEFT_BRACKET || key == GLFW_KEY_RIGHT_BRACKET) && !roster.empty()) {
                const int n = int(roster.size());
                view.robot  = (view.robot + (key == GLFW_KEY_RIGHT_BRACKET ? 1 : n - 1)) % n;
                view.chaser.reset();
            }
            if (key == GLFW_KEY_M) {
                show_minimap = !show_minimap;
            }
            if (key == GLFW_KEY_H) {
                show_panel = !show_panel;
            }
            // SPACE (pause) is intentionally NOT wired here: pausing physics is
            // module::Simulation's domain (it owns the 1 kHz stepping thread), and
            // there's no pause switch exposed yet. Future work once Simulation grows
            // one — see docs/K1_MUJOCO_SETUP.md known limitations.
        }

        void mouse_button_cb(GLFWwindow* w, int button, int act, int mods) {
            button_left   = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            button_middle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            button_right  = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
            glfwGetCursorPos(w, &last_x, &last_y);

            if (act == GLFW_RELEASE) {
                end_perturb();
                if (!button_left && !button_middle && !button_right) {
                    ui_drag = false;
                }
                return;
            }
            if (imgui_wants_mouse()) {
                ui_drag = true;
                return;
            }
            if (g_model == nullptr || g_data == nullptr || g_mutex == nullptr || !scene_ready) {
                return;
            }

            drag_view = view_under(w, last_x, last_y);
            use_camera_of(drag_view);

            const bool mod_ctrl = (mods & GLFW_MOD_CONTROL) != 0;

            std::lock_guard<std::mutex> lock(*g_mutex);

            // Ctrl + drag on the already-selected body: push/twist it (get-up/fall testing).
            if (mod_ctrl && pert.select > 0) {
                if (button_right) {
                    mjv_initPerturb(g_model, g_data, &scn, &pert);
                    pert.active = mjPERT_TRANSLATE;
                }
                else if (button_left) {
                    mjv_initPerturb(g_model, g_data, &scn, &pert);
                    pert.active = mjPERT_ROTATE;
                }
            }

            // Double-click (any button, <300 ms apart): select the body under the cursor.
            static double last_click_time = 0.0;
            static int last_click_button  = -1;
            const double now              = glfwGetTime();
            const bool double_click       = (button == last_click_button) && (now - last_click_time < 0.3);
            last_click_time               = now;
            last_click_button             = button;

            if (double_click) {
                double fx = 0., fy = 0.;
                int fb_w = 0, fb_h = 0;
                to_framebuffer(w, last_x, last_y, fx, fy, fb_w, fb_h);
                const auto rects       = viewer::layout_rects(layout, fb_w, fb_h);
                const viewer::Rect& rc = rects[drag_view];
                if (rc.width > 0 && rc.height > 0) {
                    const double aspect = static_cast<double>(rc.width) / static_cast<double>(rc.height);
                    const double relx   = (fx - rc.left) / rc.width;
                    const double rely   = (fy - rc.bottom) / rc.height;

                    mjtNum selpnt[3];
                    int geomid = -1;
                    int flexid = -1;
                    int skinid = -1;
                    const int body =
                        mjv_select(g_model, g_data, &opt, aspect, relx, rely, &scn, selpnt, &geomid, &flexid, &skinid);

                    if (body >= 0) {
                        pert.select     = body;
                        pert.skinselect = skinid;
                        mjtNum tmp[3];
                        mju_sub3(tmp, selpnt, g_data->xpos + 3 * body);
                        mju_mulMatTVec3(pert.localpos, g_data->xmat + 9 * body, tmp);
                    }
                    else {
                        pert.select     = 0;
                        pert.skinselect = -1;
                        end_perturb();
                    }
                }
            }
        }

        void mouse_move_cb(GLFWwindow* w, double xpos, double ypos) {
            const double dx = xpos - last_x;
            const double dy = ypos - last_y;
            last_x          = xpos;
            last_y          = ypos;

            if (!button_left && !button_middle && !button_right) {
                return;
            }
            if (g_model == nullptr || ui_drag || !scene_ready) {
                return;
            }

            int width  = 0;
            int height = 0;
            glfwGetWindowSize(w, &width, &height);
            if (height == 0) {
                return;
            }

            const bool mod_shift =
                glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            const bool mod_ctrl = glfwGetKey(w, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS
                                  || glfwGetKey(w, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

            use_camera_of(drag_view);
            if (mod_ctrl && pert.active != 0 && g_data != nullptr && g_mutex != nullptr) {
                const mjtMouse action = button_right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
                std::lock_guard<std::mutex> lock(*g_mutex);
                mjv_movePerturb(g_model, g_data, action, dx / height, dy / height, &scn, &pert);
                return;
            }

            View& view = views[drag_view];
            if (view.mode == Mode::CHASE) {
                // Orbit around the robot while still following it: left drag swings and
                // tilts, right drag pulls the camera in and out.
                if (button_left) {
                    view.chase.yaw_offset = viewer::wrap_angle((view.chase.yaw_offset - 180. * dx / height) * M_PI / 180.)
                                            * 180. / M_PI;
                    view.chase.elevation = std::clamp(view.chase.elevation - 90. * dy / height, -89., 30.);
                }
                else if (button_right) {
                    view.chase.distance = std::clamp(view.chase.distance * std::exp(2. * dy / height), 0.3, 20.);
                }
                return;
            }
            if (view.mode == Mode::HEAD) {
                return;  // the head camera is fixed to the robot
            }

            mjtMouse action = mjMOUSE_ZOOM;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            }
            else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            }
            mjv_moveCamera(g_model, action, dx / height, dy / height, &scn, &view.cam);
        }

        void scroll_cb(GLFWwindow* w, double /*xoffset*/, double yoffset) {
            if (g_model == nullptr || imgui_wants_mouse() || !scene_ready) {
                return;
            }
            double x = 0., y = 0.;
            glfwGetCursorPos(w, &x, &y);
            const int index = view_under(w, x, y);
            View& view      = views[index];
            if (view.mode == Mode::CHASE) {
                view.chase.distance = std::clamp(view.chase.distance * std::exp(-0.1 * yoffset), 0.3, 20.);
            }
            else if (view.mode == Mode::FREE) {
                use_camera_of(index);
                mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn, &view.cam);
            }
        }

        // Point every visible view's camera at its target. Needs the sim mutex: reads mjData.
        void aim_cameras(double dt) {
            for (int i = 0; i < active_views(); ++i) {
                View& view      = views[i];
                const int robot = std::clamp(view.robot, 0, std::max(0, int(roster.size()) - 1));
                const int body  = robot < int(robot_body.size()) ? robot_body[robot] : -1;
                const int head  = robot < int(robot_head.size()) ? robot_head[robot] : -1;

                if (view.mode == Mode::HEAD && head >= 0) {
                    view.cam.type       = mjCAMERA_FIXED;
                    view.cam.fixedcamid = head;
                }
                else if (view.mode != Mode::FREE && body >= 0) {
                    // CHASE, or HEAD on a model without a head camera.
                    const std::array<double, 3> trunk{g_data->xpos[3 * body],
                                                      g_data->xpos[3 * body + 1],
                                                      g_data->xpos[3 * body + 2]};
                    const auto pose = view.chaser.update(view.chase,
                                                         trunk,
                                                         viewer::heading_from_xmat(g_data->xmat + 9 * body),
                                                         dt);
                    view.cam.type   = mjCAMERA_FREE;
                    for (int k = 0; k < 3; ++k) {
                        view.cam.lookat[k] = pose.lookat[k];
                    }
                    view.cam.distance  = pose.distance;
                    view.cam.azimuth   = pose.azimuth;
                    view.cam.elevation = pose.elevation;
                }
                else {
                    view.cam.type = mjCAMERA_FREE;
                }
            }
        }

        void draw_panel() {
            if (!show_panel) {
                return;
            }
            ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowBgAlpha(0.85f);
            if (!ImGui::Begin("Cameras", &show_panel, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::End();
                return;
            }

            const float combo_pad = ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 2.f;
            ImGui::SetNextItemWidth(ImGui::CalcTextSize("Picture in picture").x + combo_pad);
            if (ImGui::BeginCombo("Layout", viewer::layout_label(layout))) {
                for (const Layout candidate : viewer::ALL_LAYOUTS) {
                    if (ImGui::Selectable(viewer::layout_label(candidate), layout == candidate)) {
                        layout = candidate;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            ImGui::Checkbox("Minimap", &show_minimap);

            for (int i = 0; i < active_views(); ++i) {
                View& view = views[i];
                ImGui::PushID(i);
                char title[32];
                std::snprintf(title, sizeof(title), "View %d", i + 1);
                ImGui::SeparatorText(title);

                ImGui::SetNextItemWidth(ImGui::CalcTextSize("Behind robot").x + combo_pad);
                if (ImGui::BeginCombo("##mode", viewer::mode_label(view.mode))) {
                    for (const Mode candidate : viewer::ALL_MODES) {
                        if (ImGui::Selectable(viewer::mode_label(candidate), view.mode == candidate)) {
                            set_mode(view, candidate);
                        }
                    }
                    ImGui::EndCombo();
                }

                if (view.mode != Mode::FREE && !roster.empty()) {
                    const int current = std::clamp(view.robot, 0, int(roster.size()) - 1);
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(ImGui::CalcTextSize("Team 000  #00  (ROS domain 00)").x + combo_pad);
                    if (ImGui::BeginCombo("##robot", robot_label[current].c_str())) {
                        for (int r = 0; r < int(roster.size()); ++r) {
                            if (ImGui::Selectable(robot_label[r].c_str(), r == current)) {
                                view.robot = r;
                                view.chaser.reset();
                            }
                        }
                        ImGui::EndCombo();
                    }
                }

                if (view.mode == Mode::CHASE) {
                    // Camera-angle buttons: where the camera sits around the robot.
                    struct Angle {
                        const char* label;
                        double yaw;
                    };
                    constexpr std::array<Angle, 4> angles{{{"Behind", 0.}, {"Left", -90.}, {"Right", 90.}, {"Front", 180.}}};
                    for (std::size_t a = 0; a < angles.size(); ++a) {
                        if (a > 0) {
                            ImGui::SameLine();
                        }
                        if (ImGui::Button(angles[a].label)) {
                            view.chase.yaw_offset = angles[a].yaw;
                        }
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("High")) {
                        view.chase.elevation = -45.;
                        view.chase.distance  = std::max(view.chase.distance, 3.);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Low")) {
                        view.chase.elevation = -10.;
                        view.chase.distance  = 2.2;
                    }
                    if (ImGui::TreeNode("Fine tune")) {
                        const double dist_min = 0.3, dist_max = 10., el_min = -89., el_max = 30.;
                        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12.f);
                        ImGui::SliderScalar("Distance (m)", ImGuiDataType_Double, &view.chase.distance, &dist_min, &dist_max, "%.1f");
                        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12.f);
                        ImGui::SliderScalar("Pitch (deg)", ImGuiDataType_Double, &view.chase.elevation, &el_min, &el_max, "%.0f");
                        ImGui::TreePop();
                    }
                }
                else if (view.mode == Mode::FREE) {
                    if (ImGui::Button("Field overview")) {
                        field_overview(view.cam);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Reset")) {
                        mjv_defaultFreeCamera(g_model, &view.cam);
                    }
                }
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::TextDisabled("Drag orbit  Scroll zoom  C mode");
            ImGui::TextDisabled("[ ] robot  V layout  M minimap  H hide panel");
            ImGui::End();
        }

        void draw_view_labels(const std::vector<viewer::Rect>& rects) {
            for (int i = 0; i < active_views(); ++i) {
                const View& view = views[i];
                const viewer::Rect& rc = rects[i];
                char label[96];
                if (view.mode == Mode::FREE || roster.empty()) {
                    std::snprintf(label, sizeof(label), "%d  %s", i + 1, viewer::mode_label(view.mode));
                }
                else {
                    const int robot = std::clamp(view.robot, 0, int(roster.size()) - 1);
                    std::snprintf(label,
                                  sizeof(label),
                                  "%d  %s  %s",
                                  i + 1,
                                  viewer::mode_label(view.mode),
                                  robot_label[robot].c_str());
                }
                mjr_overlay(mjFONT_NORMAL,
                            mjGRID_BOTTOMLEFT,
                            mjrRect{rc.left, rc.bottom, rc.width, rc.height},
                            label,
                            nullptr,
                            &con);
            }
        }

        // Draws the minimap in the top-right corner; returns the y (from the bottom) of its lower
        // edge, or the window height when it's hidden, so the telemetry can sit beneath it.
        int draw_minimap(int width, int height) {
            if (!show_minimap || minimap_extent[0] <= 0. || minimap_extent[1] <= 0.) {
                return height;
            }
            const int margin = std::max(4, std::min(width, height) / 80);
            const int map_w  = int(width * minimap_width);
            const int map_h  = int(map_w * minimap_extent[1] / minimap_extent[0]);
            const mjrRect rect{width - map_w - margin, height - map_h - margin, map_w, map_h};
            mjr_rectangle(mjrRect{rect.left - 2, rect.bottom - 2, rect.width + 4, rect.height + 4}, .9f, .9f, .9f, 1.f);

            // Straight down from 20 m, field +x to the right and +y up.
            mjvGLCamera top{};
            top.pos[2]         = 20.f;
            top.forward[2]     = -1.f;
            top.up[1]          = 1.f;
            top.frustum_bottom = float(-minimap_extent[1] / 2.);
            top.frustum_top    = float(minimap_extent[1] / 2.);
            top.frustum_width  = float(minimap_extent[0] / 2.);  // mjr_render reads this as the half-width
            top.frustum_near   = 1.f;
            top.frustum_far    = 40.f;
            top.orthographic   = 1;

            // Markers live in the scene only for this render: a disc in the team colour, an arrow
            // along the robot's heading, and the numbers of the views following it.
            const int saved_ngeom = scn.ngeom;
            const float blue[4] = {.2f, .55f, 1.f, 1.f}, red[4] = {1.f, .3f, .25f, 1.f}, white[4] = {1.f, 1.f, 1.f, 1.f};
            const mjtNum eye[9]  = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
            for (std::size_t r = 0; r < roster.size() && r < robot_pose.size(); ++r) {
                if (r >= robot_body.size() || robot_body[r] < 0 || scn.ngeom + 3 > scn.maxgeom) {
                    continue;
                }
                const auto& p      = robot_pose[r];
                const float* color = roster[r].team_id == roster.front().team_id ? blue : red;

                const mjtNum disc_size[3] = {0.28, 0.28, 0.02};
                const mjtNum disc_pos[3]  = {p[0], p[1], 1.0};
                mjv_initGeom(&scn.geoms[scn.ngeom++], mjGEOM_CYLINDER, disc_size, disc_pos, eye, color);

                mjvGeom& arrow = scn.geoms[scn.ngeom++];
                // A heading stick from the disc centre (an mjGEOM_ARROW barely shows from overhead).
                mjv_initGeom(&arrow, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, white);
                const mjtNum from[3] = {p[0], p[1], 1.1};
                const mjtNum to[3]   = {p[0] + 0.7 * std::cos(p[2]), p[1] + 0.7 * std::sin(p[2]), 1.1};
                mjv_connector(&arrow, mjGEOM_CAPSULE, 0.07, from, to);

                std::string followers;
                for (int i = 0; i < active_views(); ++i) {
                    if (views[i].mode != Mode::FREE && views[i].robot == int(r)) {
                        followers += (followers.empty() ? "" : ",") + std::to_string(i + 1);
                    }
                }
                if (!followers.empty()) {
                    const mjtNum zero[3]     = {0., 0., 0.};
                    // Beside the marker on the field-centre side, so it isn't clipped at the edge.
                    const mjtNum label_pos[3] = {p[0] + (p[0] > 0. ? -0.7 : 0.4), p[1] + (p[1] > 0. ? -0.5 : 0.7), 1.2};
                    mjvGeom& label            = scn.geoms[scn.ngeom++];
                    mjv_initGeom(&label, mjGEOM_LABEL, zero, label_pos, eye, white);
                    std::snprintf(label.label, sizeof(label.label), "%s", followers.c_str());
                }
            }

            scn.camera[0] = top;
            scn.camera[1] = top;
            mjr_render(rect, &scn, &con);
            scn.ngeom = saved_ngeom;
            return rect.bottom - margin;
        }

        // Default view setup from viewer.yaml (optional; built-in defaults otherwise).
        // Returns an error message, empty if the file is fine or absent.
        std::string load_view_config(int& window_w, int& window_h) {
            YAML::Node cfg;
            try {
                cfg = config::load("viewer.yaml");
            }
            catch (const YAML::BadFile&) {
                return "";
            }
            try {
                if (cfg["layout"]) {
                    layout = viewer::parse_layout(cfg["layout"].as<std::string>());
                }
                if (cfg["show_panel"]) {
                    show_panel = cfg["show_panel"].as<bool>();
                }
                if (const auto mm = cfg["minimap"]) {
                    show_minimap = mm["enabled"].as<bool>(show_minimap);
                    if (mm["extent"]) {
                        minimap_extent = {mm["extent"][0].as<double>(), mm["extent"][1].as<double>()};
                    }
                    minimap_width = std::clamp(mm["width"].as<double>(minimap_width), 0.05, 0.6);
                }
                if (cfg["window"]) {
                    window_w = cfg["window"][0].as<int>();
                    window_h = cfg["window"][1].as<int>();
                }
                viewer::ChaseParams chase;
                if (const auto c = cfg["chase"]) {
                    chase.distance  = c["distance"].as<double>(chase.distance);
                    chase.elevation = c["elevation"].as<double>(chase.elevation);
                    chase.height    = c["height"].as<double>(chase.height);
                    chase.smoothing = c["smoothing"].as<double>(chase.smoothing);
                }
                for (auto& view : views) {
                    view.chase = chase;
                }
                if (const auto list = cfg["views"]) {
                    for (std::size_t i = 0; i < list.size() && i < views.size(); ++i) {
                        views[i].mode  = viewer::parse_mode(list[i]["mode"].as<std::string>("chase"));
                        views[i].robot = list[i]["robot"].as<int>(0);
                    }
                }
            }
            catch (const std::exception& error) {
                return error.what();
            }
            return "";
        }

    }  // namespace

    Viewer::Viewer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Bridge for the plain-C GLFW key callback (Backspace = sim reset). emit() is
        // thread-safe, and the callback only ever runs on the MainThread glfwPollEvents pump.
        g_request_reset = [this] { emit(std::make_unique<message::SimResetRequest>()); };

        on<Startup, MainThread>().then([this] {
            if (cli().headless) {
                log<NUClear::LogLevel::INFO>("Viewer disabled (--headless)");
                return;
            }

            // Built-in defaults: two behind-the-robot views side by side (the main robot and,
            // in a match, the other team's first player), then a free and a head view for the grid.
            roster = config::robot_roster();
            views[0].mode  = Mode::CHASE;
            views[0].robot = 0;
            views[1].mode  = roster.size() > 1 ? Mode::CHASE : Mode::FREE;
            views[1].robot = roster.size() > 1 ? int(roster.size() / 2) : 0;
            views[2].mode  = Mode::FREE;
            views[3].mode  = Mode::HEAD;
            int window_w = 1600, window_h = 900;
            if (const std::string error = load_view_config(window_w, window_h); !error.empty()) {
                log<NUClear::LogLevel::ERROR>("viewer.yaml:", error, "— some camera defaults are built-in");
            }
            for (auto& view : views) {
                // A view of a robot that isn't in this run (e.g. robot 1 with one robot) shows
                // the free camera instead of a second copy of robot 0.
                if (view.robot >= int(roster.size()) || view.robot < 0) {
                    view.mode  = Mode::FREE;
                    view.robot = 0;
                }
            }
            for (const auto& robot : roster) {
                // The robot's DDS domain is the ROS_DOMAIN_ID its controller (NUbots / a ROS 2
                // node) must use to talk to it.
                char name[48], label[80];
                std::snprintf(name, sizeof(name), "Team %d  #%d", robot.team_id, robot.player_id);
                robot_name.emplace_back(roster.size() > 1 ? name : "Main robot");
                std::snprintf(label, sizeof(label), "%s  (ROS domain %d)", robot_name.back().c_str(), robot.dds_domain);
                robot_label.emplace_back(label);
            }
            robot_pose.assign(roster.size(), {0., 0., 0.});

            if (glfwInit() == GLFW_FALSE) {
                log<NUClear::LogLevel::ERROR>("glfwInit() failed — continuing without a viewer window");
                return;
            }

            window = glfwCreateWindow(window_w, window_h, "K1 MuJoCo Sim", nullptr, nullptr);
            if (window == nullptr) {
                log<NUClear::LogLevel::ERROR>("glfwCreateWindow() failed — continuing without a viewer window");
                glfwTerminate();
                return;
            }

            glfwMakeContextCurrent(window);
            // No vsync: frame pacing is already owned by the Every<16ms> reaction
            // below, and blocking glfwSwapBuffers() on a vblank the driver never
            // delivers (headless/virtual displays, some remote X servers) would
            // stall the NUClear MainThread pool — the same thread glfwPollEvents
            // needs, so the whole viewer (and its window-close handling) hangs.
            glfwSwapInterval(0);

            for (auto& view : views) {
                mjv_defaultCamera(&view.cam);
            }
            mjv_defaultCamera(&video_cam);
            mjv_defaultOption(&opt);
            mjv_defaultPerturb(&pert);
            mjr_defaultContext(&con);
            // mjv_makeScene/mjr_makeContext need an mjModel, so they're deferred
            // to the Trigger<SimHandles> reaction below (also MainThread — GL
            // context creation is thread-bound to whichever thread called
            // glfwMakeContextCurrent, i.e. this one).

            glfwSetMouseButtonCallback(window, mouse_button_cb);
            glfwSetCursorPosCallback(window, mouse_move_cb);
            glfwSetScrollCallback(window, scroll_cb);
            glfwSetKeyCallback(window, keyboard_cb);

            // ImGui installs its own GLFW callbacks and chains to the ones set above, so it
            // must come after them. Each of ours checks WantCaptureMouse/Keyboard first.
            IMGUI_CHECKVERSION();
            ImGui::CreateContext();
            ImGuiIO& io    = ImGui::GetIO();
            io.IniFilename = nullptr;  // don't drop an imgui.ini into the working directory
            float scale_x = 1.f, scale_y = 1.f;
            glfwGetWindowContentScale(window, &scale_x, &scale_y);
            const float ui_scale = std::max(1.f, scale_x);
            ImGui::StyleColorsDark();
            ImGui::GetStyle().ScaleAllSizes(ui_scale);
            io.FontGlobalScale = ui_scale;
            ImGui_ImplGlfw_InitForOpenGL(window, true);
            ImGui_ImplOpenGL2_Init();
            imgui_ready = true;

            log<NUClear::LogLevel::INFO>("Viewer window created (",
                                         window_w,
                                         "x",
                                         window_h,
                                         ") — layout",
                                         viewer::layout_name(layout),
                                         "— waiting for the simulation model");
        });

        on<Trigger<message::SimHandles>, MainThread>().then([this](const message::SimHandles& handles) {
            if (cli().headless) {
                return;
            }

            g_model        = handles.model;
            g_data         = handles.data;
            g_mutex        = handles.mutex;
            g_measured_rtf = handles.measured_rtf;

            if (window != nullptr && !scene_ready && g_model != nullptr) {
                mjv_makeScene(g_model, &scn, 2000);
                mjr_makeContext(g_model, &con, mjFONTSCALE_150);
                // mjv_defaultCamera (Startup) doesn't know the model's scale; now
                // that it's loaded, frame the free cameras on it (lookat/distance
                // from the compiled model extent) so the scene isn't clipped or
                // too distant to see on first frame.
                for (auto& view : views) {
                    mjv_defaultFreeCamera(g_model, &view.cam);
                }
                mjv_defaultFreeCamera(g_model, &video_cam);

                robot_body.clear();
                robot_head.clear();
                for (const auto& robot : roster) {
                    const std::string prefix = robot_prefix(robot.model_index);
                    robot_body.push_back(mj_name2id(g_model, mjOBJ_BODY, robot.body().c_str()));
                    robot_head.push_back(mj_name2id(g_model, mjOBJ_CAMERA, (prefix + "head").c_str()));
                    if (robot_body.back() < 0) {
                        log<NUClear::LogLevel::WARN>("Viewer: no body", robot.body(), "— its views fall back to free");
                    }
                }

                scene_ready = true;
                log<NUClear::LogLevel::INFO>("Viewer scene ready (",
                                             g_model->nbody,
                                             "bodies,",
                                             g_model->ngeom,
                                             "geoms)");
            }
        });

        on<Trigger<message::VideoRecordRequest>, MainThread>().then([this](const message::VideoRecordRequest& request) {
            if (!scene_ready || video) {
                log<NUClear::LogLevel::ERROR>("Video requires a ready viewer and no existing recording");
                return;
            }
            try {
                video_roster = config::robot_roster();
                video        = std::make_unique<VideoRecorder>(request.path);
                video_rgb.resize(VideoRecorder::width * VideoRecorder::height * 3);
                mjr_resizeOffscreen(VideoRecorder::width, VideoRecorder::height, &con);
                // The recording has its own fixed broadcast camera; the window's views are untouched.
                video_cam.type      = mjCAMERA_FREE;
                video_cam.lookat[0] = video_cam.lookat[1] = video_cam.lookat[2] = 0.;
                video_cam.distance                                              = 12.5;
                video_cam.azimuth                                               = 90.;
                video_cam.elevation                                             = -80.;
                video_frames                                                    = 0;
                video_limit = int(std::ceil(request.duration * VideoRecorder::fps));
                video_start = NUClear::clock::now();
                log<NUClear::LogLevel::INFO>("VIDEO started",
                                             request.path,
                                             "frames",
                                             video_limit,
                                             "encoder_pid",
                                             video->pid());
            }
            catch (const std::exception& error) {
                video.reset();
                log<NUClear::LogLevel::ERROR>("VIDEO failed", error.what());
            }
        });

        // Overlay-only telemetry — never touches mjData, so no MainThread/mutex needed.
        on<Trigger<message::SimStateUpdate>>().then([](const message::SimStateUpdate& state) {
            g_sim_time.store(state.sim_time, std::memory_order_relaxed);
            g_mode.store(state.mode, std::memory_order_relaxed);
        });

        on<Every<16, std::chrono::milliseconds>, MainThread>().then([this] {
            if (cli().headless || window == nullptr) {
                return;
            }

            if (glfwWindowShouldClose(window) != 0) {
                powerplant.shutdown();
                return;
            }

            glfwPollEvents();

            const double now = glfwGetTime();
            const double dt  = last_frame > 0. ? std::min(now - last_frame, 0.25) : 0.;
            last_frame       = now;

            if (scene_ready && g_data != nullptr && g_mutex != nullptr) {
                // Lock only around the mjData touch points: applying the perturb
                // force/clearing it, aiming the cameras, and mjv_updateScene's read of
                // mjData. All are sub-millisecond; the physics thread never blocks on us
                // for longer than that.
                std::lock_guard<std::mutex> lock(*g_mutex);
                if (pert.select > 0) {
                    if (pert.active != 0) {
                        mjv_applyPerturbForce(g_model, g_data, &pert);
                    }
                    else {
                        mju_zero(g_data->xfrc_applied + 6 * pert.select, 6);
                    }
                }
                aim_cameras(dt);
                for (std::size_t r = 0; r < robot_body.size(); ++r) {
                    if (robot_body[r] >= 0) {
                        const int b   = robot_body[r];
                        robot_pose[r] = {g_data->xpos[3 * b],
                                         g_data->xpos[3 * b + 1],
                                         viewer::heading_from_xmat(g_data->xmat + 9 * b)};
                    }
                }
                // The geometry is shared by every view; only the camera differs, so build
                // the scene once and take each view's camera with mjv_updateCamera.
                mjv_updateScene(g_model, g_data, &opt, &pert, &views[0].cam, mjCAT_ALL, &scn);
                for (int i = 0; i < active_views(); ++i) {
                    mjv_updateCamera(g_model, g_data, &views[i].cam, &scn);
                    views[i].gl = {scn.camera[0], scn.camera[1]};
                }
                if (video) {
                    mjv_updateCamera(g_model, g_data, &video_cam, &scn);
                    video_gl = {scn.camera[0], scn.camera[1]};
                    for (const auto& robot : video_roster) {
                        const int body = mj_name2id(g_model, mjOBJ_BODY, robot.body().c_str());
                        if (body < 0 || scn.ngeom >= scn.maxgeom)
                            continue;
                        const mjtNum size[3] = {0., 0., 0.};
                        const mjtNum pos[3]  = {g_data->xpos[3 * body],
                                                g_data->xpos[3 * body + 1],
                                                g_data->xpos[3 * body + 2] + .5};
                        const mjtNum mat[9]  = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
                        const float blue[4] = {.2f, .55f, 1.f, 1.f}, red[4] = {1.f, .3f, .25f, 1.f};
                        auto& label = scn.geoms[scn.ngeom++];
                        mjv_initGeom(&label,
                                     mjGEOM_LABEL,
                                     size,
                                     pos,
                                     mat,
                                     robot.team_id == video_roster.front().team_id ? blue : red);
                        std::snprintf(label.label, sizeof(label.label), "%d/%d", robot.team_id, robot.player_id);
                    }
                }
            }

            int width  = 0;
            int height = 0;
            glfwGetFramebufferSize(window, &width, &height);
            const mjrRect viewport{0, 0, width, height};

            if (scene_ready) {
                const auto rects = viewer::layout_rects(layout, width, height);
                for (int i = 0; i < active_views(); ++i) {
                    const viewer::Rect& rc = rects[i];
                    if (layout == Layout::PICTURE_IN_PICTURE && i > 0) {
                        // A thin frame so the inset reads as separate from the main view.
                        const float white[4] = {.9f, .9f, .9f, 1.f};
                        mjr_rectangle(mjrRect{rc.left - 2, rc.bottom - 2, rc.width + 4, rc.height + 4},
                                      white[0],
                                      white[1],
                                      white[2],
                                      white[3]);
                    }
                    use_camera_of(i);
                    mjr_render(mjrRect{rc.left, rc.bottom, rc.width, rc.height}, &scn, &con);
                }
                if (layout != Layout::SINGLE && layout != Layout::PICTURE_IN_PICTURE) {
                    // Dividers between tiled views.
                    for (int i = 1; i < active_views(); ++i) {
                        const viewer::Rect& rc = rects[i];
                        if (rc.left > 0) {
                            mjr_rectangle(mjrRect{rc.left - 1, rc.bottom, 2, rc.height}, .1f, .1f, .1f, 1.f);
                        }
                        if (rc.bottom + rc.height < height) {
                            mjr_rectangle(mjrRect{rc.left, rc.bottom + rc.height - 1, rc.width, 2}, .1f, .1f, .1f, 1.f);
                        }
                    }
                }
                draw_view_labels(rects);
                const int minimap_bottom = draw_minimap(width, height);

                const double rtf = (g_measured_rtf != nullptr) ? g_measured_rtf->load(std::memory_order_relaxed) : 0.0;
                char overlay[256];
                std::snprintf(overlay,
                              sizeof(overlay),
                              "sim time: %.1f s\nRTF: %.2f\nmode: %s",
                              g_sim_time.load(std::memory_order_relaxed),
                              rtf,
                              mode_name(g_mode.load(std::memory_order_relaxed)));
                // Telemetry sits in the top-right corner, under the minimap when it's shown.
                mjr_overlay(mjFONT_NORMAL,
                            mjGRID_TOPRIGHT,
                            mjrRect{0, 0, width, minimap_bottom},
                            overlay,
                            nullptr,
                            &con);
            }

            if (scene_ready && video) {
                const int due = std::min(
                    video_limit,
                    int(std::chrono::duration<double>(NUClear::clock::now() - video_start).count() * VideoRecorder::fps)
                        + 1);
                if (due > video_frames) {
                    mjr_setBuffer(mjFB_OFFSCREEN, &con);
                    const mjrRect recording{0, 0, VideoRecorder::width, VideoRecorder::height};
                    scn.camera[0] = video_gl[0];
                    scn.camera[1] = video_gl[1];
                    mjr_render(recording, &scn, &con);
                    char caption[128];
                    std::snprintf(caption,
                                  sizeof(caption),
                                  "NUSim native 3v3 | six independent players\nsim time %.1f s | RTF %.2f",
                                  g_sim_time.load(),
                                  g_measured_rtf ? g_measured_rtf->load() : 0.);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, recording, caption, nullptr, &con);
                    mjr_readPixels(video_rgb.data(), nullptr, recording, &con);
                    mjr_setBuffer(mjFB_WINDOW, &con);
                    try {
                        while (video_frames < due) {
                            video->write(video_rgb);
                            ++video_frames;
                        }
                        if (video_frames == video_limit) {
                            const bool ok = video->finish();
                            video.reset();
                            log<NUClear::LogLevel::INFO>("VIDEO finished", video_frames, "frames, encoder success", ok);
                        }
                    }
                    catch (const std::exception& error) {
                        video.reset();
                        log<NUClear::LogLevel::ERROR>("VIDEO failed", error.what());
                    }
                }
            }

            if (imgui_ready) {
                // The on-screen buttons, drawn last so they sit over every view.
                ImGui_ImplOpenGL2_NewFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();
                if (scene_ready) {
                    draw_panel();
                }
                ImGui::Render();
                ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
            }

            glfwSwapBuffers(window);
        });

        on<Shutdown, MainThread>().then([this] {
            if (!cli().headless) {
                if (video) {
                    video->finish();
                    video.reset();
                }
                if (imgui_ready) {
                    ImGui_ImplOpenGL2_Shutdown();
                    ImGui_ImplGlfw_Shutdown();
                    ImGui::DestroyContext();
                    imgui_ready = false;
                }
                if (scene_ready) {
                    mjv_freeScene(&scn);
                    mjr_freeContext(&con);
                    scene_ready = false;
                }
                if (window != nullptr) {
                    glfwDestroyWindow(window);
                    window = nullptr;
                }
                glfwTerminate();
            }
            log<NUClear::LogLevel::INFO>("Viewer shutting down");
        });
    }

}  // namespace k1sim::module
