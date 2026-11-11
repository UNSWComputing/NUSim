#ifndef K1SIM_MODULE_VIEWER_VIEWCAMERA_HPP
#define K1SIM_MODULE_VIEWER_VIEWCAMERA_HPP

#include <array>
#include <string>
#include <vector>

// GL-free pieces of the viewer's multi-view cameras: how the window splits into views, and the
// smoothed third-person "chase" camera that sits behind a robot. Kept apart from Viewer.cpp so
// they can be unit tested without a window.
namespace k1sim::viewer {

    inline constexpr int MAX_VIEWS = 4;

    enum class Layout { SINGLE, SIDE_BY_SIDE, STACKED, PICTURE_IN_PICTURE, GRID };
    inline constexpr std::array<Layout, 5> ALL_LAYOUTS{Layout::SINGLE,
                                                       Layout::SIDE_BY_SIDE,
                                                       Layout::STACKED,
                                                       Layout::PICTURE_IN_PICTURE,
                                                       Layout::GRID};

    // CHASE follows a robot from behind (third-person game camera); FREE is the mouse-driven
    // simulate-style camera; HEAD is the robot's own MJCF "head" camera.
    enum class Mode { CHASE, FREE, HEAD };
    inline constexpr std::array<Mode, 3> ALL_MODES{Mode::CHASE, Mode::FREE, Mode::HEAD};

    const char* layout_name(Layout layout);  // config spelling, e.g. "side_by_side"
    const char* layout_label(Layout layout);  // UI spelling, e.g. "Side by side"
    const char* mode_name(Mode mode);
    const char* mode_label(Mode mode);
    // Throws std::invalid_argument naming the accepted spellings.
    Layout parse_layout(const std::string& name);
    Mode parse_mode(const std::string& name);

    int view_count(Layout layout);

    // A viewport in framebuffer pixels, origin bottom-left (OpenGL / mjrRect convention).
    struct Rect {
        int left   = 0;
        int bottom = 0;
        int width  = 0;
        int height = 0;
        bool contains(double x, double y) const {
            return x >= left && x < left + width && y >= bottom && y < bottom + height;
        }
    };

    // One rect per view, in draw order: a picture-in-picture inset comes after (on top of) the
    // main view.
    std::vector<Rect> layout_rects(Layout layout, int width, int height);

    // The view under framebuffer point (x, y from the bottom), the topmost one if they overlap;
    // -1 when the point is outside the window.
    int view_at(Layout layout, int width, int height, double x, double y);

    struct ChaseParams {
        double distance   = 2.5;   // m from the look-at point
        double elevation  = -15.;  // deg; negative looks down
        double height     = 0.3;   // m the look-at point sits above the trunk
        double yaw_offset = 0.;    // deg around the robot; 0 = behind, 180 = in front
        double smoothing  = 0.2;   // s time constant for following the robot; 0 = rigid
    };

    // MuJoCo free-camera parameters (mjvCamera lookat/distance/azimuth/elevation).
    struct CameraPose {
        std::array<double, 3> lookat{};
        double distance  = 0.;
        double azimuth   = 0.;
        double elevation = 0.;
    };

    class ChaseCamera {
    public:
        // Advance by dt seconds towards the robot at `trunk` heading `heading` (rad, world z).
        // The first update, and any jump over `snap_distance` (a reset or respawn), snaps.
        CameraPose update(const ChaseParams& params, const std::array<double, 3>& trunk, double heading, double dt);
        void reset() {
            initialised = false;
        }

        static constexpr double snap_distance = 1.5;

    private:
        bool initialised = false;
        std::array<double, 3> lookat{};
        double heading = 0.;  // smoothed, rad, unwrapped
    };

    // Heading (rad) of a body's +x axis projected on the ground, from a row-major 3x3 rotation.
    double heading_from_xmat(const double* xmat);

    // Wrap an angle to (-pi, pi].
    double wrap_angle(double angle);

}  // namespace k1sim::viewer

#endif  // K1SIM_MODULE_VIEWER_VIEWCAMERA_HPP
