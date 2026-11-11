// Viewer multi-view cameras: the window split for each layout, picking the view under the
// cursor, and the third-person chase camera (behind the robot, following it smoothly, snapping
// on a respawn). GL-free -- module/Viewer/src/ViewCamera.hpp.
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "module/Viewer/src/ViewCamera.hpp"

using namespace k1sim::viewer;

namespace {
    int failures = 0;

    void check(bool ok, const char* what) {
        if (!ok) {
            std::fprintf(stderr, "FAIL: %s\n", what);
            ++failures;
        }
    }

    bool near(double a, double b, double tol = 1e-6) {
        return std::fabs(a - b) < tol;
    }
}  // namespace

int main() {
    // Every layout tiles exactly its view count, and tiled layouts cover the window once.
    for (const Layout layout : ALL_LAYOUTS) {
        const auto rects = layout_rects(layout, 1601, 901);
        check(int(rects.size()) == view_count(layout), "one rect per view");
        long area = 0;
        for (const auto& rc : rects) {
            check(rc.width > 0 && rc.height > 0, "non-empty rect");
            check(rc.left >= 0 && rc.bottom >= 0 && rc.left + rc.width <= 1601 && rc.bottom + rc.height <= 901,
                  "rect inside the window");
            area += long(rc.width) * rc.height;
        }
        if (layout != Layout::PICTURE_IN_PICTURE) {
            check(area == 1601L * 901L, "tiled views cover the window exactly once");
        }
        check(parse_layout(layout_name(layout)) == layout, "layout names round-trip");
    }

    // Side by side: view 1 left, view 2 right. Stacked: view 1 on top (GL y grows upwards).
    check(view_at(Layout::SIDE_BY_SIDE, 1600, 900, 100, 450) == 0, "left half is view 1");
    check(view_at(Layout::SIDE_BY_SIDE, 1600, 900, 1500, 450) == 1, "right half is view 2");
    check(view_at(Layout::STACKED, 1600, 900, 800, 800) == 0, "top is view 1");
    check(view_at(Layout::STACKED, 1600, 900, 800, 100) == 1, "bottom is view 2");
    // Picture in picture: the inset (bottom-right) wins over the main view beneath it.
    check(view_at(Layout::PICTURE_IN_PICTURE, 1600, 900, 1500, 60) == 1, "inset is on top");
    check(view_at(Layout::PICTURE_IN_PICTURE, 1600, 900, 100, 800) == 0, "elsewhere is the main view");
    check(view_at(Layout::GRID, 1600, 900, 1500, 100) == 3, "grid bottom-right is view 4");
    check(view_at(Layout::SINGLE, 1600, 900, -1, 10) == -1, "outside the window");

    for (const Mode mode : ALL_MODES) {
        check(parse_mode(mode_name(mode)) == mode, "mode names round-trip");
    }
    bool threw = false;
    try {
        parse_mode("orbit");
    }
    catch (const std::invalid_argument&) {
        threw = true;
    }
    check(threw, "unknown mode is rejected");

    // Heading from a body rotation: yaw of 90 deg about z.
    const double yaw90[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    check(near(heading_from_xmat(yaw90), M_PI / 2), "heading of a +90 deg yaw");
    check(near(wrap_angle(3 * M_PI / 2), -M_PI / 2), "wrap past +pi");
    check(near(wrap_angle(-M_PI), M_PI), "wrap -pi to +pi");

    // The chase camera's first update snaps behind the robot: MuJoCo places a free camera at
    // lookat - distance * (cos el cos az, cos el sin az, sin el), so azimuth == heading puts it
    // behind the robot looking the way it faces.
    ChaseParams params;
    params.distance  = 2.;
    params.elevation = -15.;
    params.height    = 0.1;
    params.smoothing = 0.2;
    ChaseCamera chase;
    auto pose = chase.update(params, {1., 2., 0.5}, M_PI / 2, 0.016);
    check(near(pose.azimuth, 90.), "behind a robot facing +y");
    check(near(pose.lookat[0], 1.) && near(pose.lookat[1], 2.) && near(pose.lookat[2], 0.6), "look-at above trunk");
    check(near(pose.distance, 2.) && near(pose.elevation, -15.), "distance and pitch pass through");

    // "Front" puts the camera on the other side, facing the robot.
    params.yaw_offset = 180.;
    pose              = chase.update(params, {1., 2., 0.5}, M_PI / 2, 0.016);
    check(near(pose.azimuth, -90.), "front view faces the robot");
    params.yaw_offset = 0.;

    // A small move is followed smoothly (part way after one frame, nearly there after 2 s), and
    // the heading turns the short way across +-180 deg.
    ChaseCamera turner;
    turner.update(params, {0., 0., 0.5}, M_PI - 0.1, 0.016);
    pose = turner.update(params, {0.2, 0., 0.5}, -M_PI + 0.1, 0.016);
    check(pose.lookat[0] > 0. && pose.lookat[0] < 0.2, "lags behind a small move");
    check(std::fabs(pose.azimuth) > 170., "turns the short way across 180 deg");
    for (int i = 0; i < 125; ++i) {
        pose = turner.update(params, {0.2, 0., 0.5}, -M_PI + 0.1, 0.016);
    }
    check(near(pose.lookat[0], 0.2, 1e-3), "catches up with the robot");
    check(near(pose.azimuth, (-M_PI + 0.1) * 180. / M_PI, 0.1), "catches up with the heading");

    // A respawn across the field snaps instead of swooping.
    pose = turner.update(params, {5., 3., 0.5}, 0., 0.016);
    check(near(pose.lookat[0], 5.) && near(pose.azimuth, 0.), "snaps after a jump");

    if (failures == 0) {
        std::printf("test_viewer_cameras: all checks passed\n");
    }
    return failures == 0 ? 0 : 1;
}
