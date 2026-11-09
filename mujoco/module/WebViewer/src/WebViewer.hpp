#ifndef K1SIM_MODULE_WEBVIEWER_HPP
#define K1SIM_MODULE_WEBVIEWER_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>

#include "module/WebViewer/src/WebStream.hpp"
#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

    // Browser based 3D viewer and match recorder (--web / --record).
    //
    // Serves the viewer page over HTTP and streams the body poses to it at half the state publish rate (25 Hz by
    // default; the viewer interpolates in between). Unlike module::Viewer it needs no GL context, so it also works
    // under --headless and from another machine. The same stream can be written to a recording file and replayed in
    // the viewer later. Inactive unless --web or --record is given.
    class WebViewer : public NUClear::Reactor {
    public:
        explicit WebViewer(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::unique_ptr<web::WebStream> stream_;
        std::mutex mutex_;  // guards handles_ and updates_ (reactions may run on different pool threads)
        message::SimHandles handles_{};
        uint64_t updates_ = 0;
        std::atomic<bool> can_walk_{false};           // the sim has a built-in walk policy
        std::atomic<int> robot_mode_{0};              // the robot's current mode (booster::RobotMode)
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_WEBVIEWER_HPP
