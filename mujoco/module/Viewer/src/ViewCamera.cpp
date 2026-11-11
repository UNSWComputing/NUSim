#include "module/Viewer/src/ViewCamera.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace k1sim::viewer {

    namespace {
        constexpr double PI = 3.14159265358979323846;
    }

    const char* layout_name(Layout layout) {
        switch (layout) {
            case Layout::SINGLE: return "single";
            case Layout::SIDE_BY_SIDE: return "side_by_side";
            case Layout::STACKED: return "stacked";
            case Layout::PICTURE_IN_PICTURE: return "picture_in_picture";
            case Layout::GRID: return "grid";
        }
        return "?";
    }

    const char* layout_label(Layout layout) {
        switch (layout) {
            case Layout::SINGLE: return "Single";
            case Layout::SIDE_BY_SIDE: return "Side by side";
            case Layout::STACKED: return "Stacked";
            case Layout::PICTURE_IN_PICTURE: return "Picture in picture";
            case Layout::GRID: return "2 x 2 grid";
        }
        return "?";
    }

    const char* mode_name(Mode mode) {
        switch (mode) {
            case Mode::CHASE: return "chase";
            case Mode::FREE: return "free";
            case Mode::HEAD: return "head";
        }
        return "?";
    }

    const char* mode_label(Mode mode) {
        switch (mode) {
            case Mode::CHASE: return "Behind robot";
            case Mode::FREE: return "Free";
            case Mode::HEAD: return "Head camera";
        }
        return "?";
    }

    Layout parse_layout(const std::string& name) {
        std::string known;
        for (const Layout layout : ALL_LAYOUTS) {
            if (name == layout_name(layout)) {
                return layout;
            }
            known += std::string(" ") + layout_name(layout);
        }
        throw std::invalid_argument("unknown viewer layout '" + name + "' (known:" + known + ")");
    }

    Mode parse_mode(const std::string& name) {
        std::string known;
        for (const Mode mode : ALL_MODES) {
            if (name == mode_name(mode)) {
                return mode;
            }
            known += std::string(" ") + mode_name(mode);
        }
        throw std::invalid_argument("unknown viewer camera mode '" + name + "' (known:" + known + ")");
    }

    int view_count(Layout layout) {
        switch (layout) {
            case Layout::SINGLE: return 1;
            case Layout::SIDE_BY_SIDE:
            case Layout::STACKED:
            case Layout::PICTURE_IN_PICTURE: return 2;
            case Layout::GRID: return 4;
        }
        return 1;
    }

    std::vector<Rect> layout_rects(Layout layout, int width, int height) {
        const int half_w = width / 2;
        const int half_h = height / 2;
        switch (layout) {
            case Layout::SINGLE: return {{0, 0, width, height}};
            case Layout::SIDE_BY_SIDE: return {{0, 0, half_w, height}, {half_w, 0, width - half_w, height}};
            case Layout::STACKED: return {{0, half_h, width, height - half_h}, {0, 0, width, half_h}};
            case Layout::PICTURE_IN_PICTURE: {
                const int inset_w = width * 32 / 100;
                const int inset_h = height * 32 / 100;
                const int margin  = std::max(4, std::min(width, height) / 60);
                return {{0, 0, width, height}, {width - inset_w - margin, margin, inset_w, inset_h}};
            }
            case Layout::GRID:
                return {{0, half_h, half_w, height - half_h},
                        {half_w, half_h, width - half_w, height - half_h},
                        {0, 0, half_w, half_h},
                        {half_w, 0, width - half_w, half_h}};
        }
        return {{0, 0, width, height}};
    }

    int view_at(Layout layout, int width, int height, double x, double y) {
        const auto rects = layout_rects(layout, width, height);
        for (int i = static_cast<int>(rects.size()) - 1; i >= 0; --i) {
            if (rects[i].contains(x, y)) {
                return i;
            }
        }
        return -1;
    }

    double wrap_angle(double angle) {
        angle = std::fmod(angle + PI, 2. * PI);
        if (angle <= 0.) {
            angle += 2. * PI;
        }
        return angle - PI;
    }

    double heading_from_xmat(const double* xmat) {
        // Column 0 of the rotation is the body +x axis in world coordinates.
        return std::atan2(xmat[3], xmat[0]);
    }

    CameraPose ChaseCamera::update(const ChaseParams& params,
                                   const std::array<double, 3>& trunk,
                                   double target_heading,
                                   double dt) {
        const std::array<double, 3> target{trunk[0], trunk[1], trunk[2] + params.height};
        const double jump = std::hypot(target[0] - lookat[0], target[1] - lookat[1], target[2] - lookat[2]);
        if (!initialised || jump > snap_distance || params.smoothing <= 0.) {
            lookat      = target;
            heading     = target_heading;
            initialised = true;
        }
        else {
            const double alpha = 1. - std::exp(-std::max(dt, 0.) / params.smoothing);
            for (int i = 0; i < 3; ++i) {
                lookat[i] += alpha * (target[i] - lookat[i]);
            }
            heading += alpha * wrap_angle(target_heading - heading);
        }

        CameraPose pose;
        pose.lookat    = lookat;
        pose.distance  = params.distance;
        pose.azimuth   = wrap_angle(heading + params.yaw_offset * PI / 180.) * 180. / PI;
        pose.elevation = params.elevation;
        return pose;
    }

}  // namespace k1sim::viewer
