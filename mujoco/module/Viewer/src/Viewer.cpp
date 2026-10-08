#include "module/Viewer/src/Viewer.hpp"

#include <GLFW/glfw3.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>

#include "VideoRecorder.hpp"
#include "shared/CliOptions.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/RobotRoster.hpp"

namespace k1sim::module {

    namespace {

        // GLFW callbacks must be plain C function pointers, and this Reactor is only
        // ever installed once (PowerPlant::install<Viewer>() is called exactly once
        // from main.cpp), so the GL/mjv/mjr state lives here as translation-unit
        // globals rather than as Viewer members — that keeps GLFW/MuJoCo headers out
        // of Viewer.hpp (main.cpp only needs the Reactor type, matching every other
        // module in this tree).

        GLFWwindow* window = nullptr;

        mjvCamera cam;
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

        // Mouse state for the simulate-style camera + perturb controls.
        bool button_left   = false;
        bool button_middle = false;
        bool button_right  = false;
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

        // Release any perturbation force/selection so a stray drag never leaves the
        // robot pinned once the mouse button comes back up.
        void end_perturb() {
            pert.active = 0;
        }

        void keyboard_cb(GLFWwindow* w, int key, int /*scancode*/, int act, int /*mods*/) {
            if (act != GLFW_PRESS) {
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

            if (g_model == nullptr || g_data == nullptr || g_mutex == nullptr) {
                return;
            }

            if (act == GLFW_RELEASE) {
                end_perturb();
                return;
            }

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
                int width  = 0;
                int height = 0;
                glfwGetWindowSize(w, &width, &height);
                if (width > 0 && height > 0) {
                    const double aspect = static_cast<double>(width) / static_cast<double>(height);
                    const double relx   = last_x / width;
                    const double rely   = 1.0 - last_y / height;

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
            if (g_model == nullptr) {
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

            if (mod_ctrl && pert.active != 0 && g_data != nullptr && g_mutex != nullptr) {
                const mjtMouse action = button_right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V;
                std::lock_guard<std::mutex> lock(*g_mutex);
                mjv_movePerturb(g_model, g_data, action, dx / height, dy / height, &scn, &pert);
                return;
            }

            mjtMouse action = mjMOUSE_ZOOM;
            if (button_right) {
                action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
            }
            else if (button_left) {
                action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
            }
            mjv_moveCamera(g_model, action, dx / height, dy / height, &scn, &cam);
        }

        void scroll_cb(GLFWwindow* /*w*/, double /*xoffset*/, double yoffset) {
            if (g_model == nullptr) {
                return;
            }
            mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn, &cam);
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

            if (glfwInit() == GLFW_FALSE) {
                log<NUClear::LogLevel::ERROR>("glfwInit() failed — continuing without a viewer window");
                return;
            }

            window = glfwCreateWindow(1200, 900, "K1 MuJoCo Sim", nullptr, nullptr);
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

            mjv_defaultCamera(&cam);
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

            log<NUClear::LogLevel::INFO>("Viewer window created (1200x900) — waiting for the simulation model");
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
                // that it's loaded, frame the free camera on it (lookat/distance
                // from the compiled model extent) so the scene isn't clipped or
                // too distant to see on first frame.
                mjv_defaultFreeCamera(g_model, &cam);
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
                cam.lookat[0] = cam.lookat[1] = cam.lookat[2] = 0.;
                cam.distance                                  = 12.5;
                cam.azimuth                                   = 90.;
                cam.elevation                                 = -80.;
                video_frames                                  = 0;
                video_limit                                   = int(std::ceil(request.duration * VideoRecorder::fps));
                video_start                                   = NUClear::clock::now();
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

            if (scene_ready && g_data != nullptr && g_mutex != nullptr) {
                // Lock only around the mjData touch points: applying the perturb
                // force/clearing it, and mjv_updateScene's read of mjData. Both
                // are sub-millisecond; the physics thread never blocks on us for
                // longer than that.
                std::lock_guard<std::mutex> lock(*g_mutex);
                if (pert.select > 0) {
                    if (pert.active != 0) {
                        mjv_applyPerturbForce(g_model, g_data, &pert);
                    }
                    else {
                        mju_zero(g_data->xfrc_applied + 6 * pert.select, 6);
                    }
                }
                mjv_updateScene(g_model, g_data, &opt, &pert, &cam, mjCAT_ALL, &scn);
                if (video) {
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
                mjr_render(viewport, &scn, &con);

                const double rtf = (g_measured_rtf != nullptr) ? g_measured_rtf->load(std::memory_order_relaxed) : 0.0;
                char overlay[256];
                std::snprintf(overlay,
                              sizeof(overlay),
                              "sim time: %.1f s\nRTF: %.2f\nmode: %s",
                              g_sim_time.load(std::memory_order_relaxed),
                              rtf,
                              mode_name(g_mode.load(std::memory_order_relaxed)));
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, overlay, nullptr, &con);
            }

            if (scene_ready && video) {
                const int due = std::min(
                    video_limit,
                    int(std::chrono::duration<double>(NUClear::clock::now() - video_start).count() * VideoRecorder::fps)
                        + 1);
                if (due > video_frames) {
                    mjr_setBuffer(mjFB_OFFSCREEN, &con);
                    const mjrRect recording{0, 0, VideoRecorder::width, VideoRecorder::height};
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

            glfwSwapBuffers(window);
        });

        on<Shutdown, MainThread>().then([this] {
            if (!cli().headless) {
                if (video) {
                    video->finish();
                    video.reset();
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
