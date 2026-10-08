#include <csignal>
#include <nuclear>

#include "module/ConsoleLog/src/ConsoleLog.hpp"
#include "module/Locomotion/src/Locomotion.hpp"
#include "module/SdkBridge/src/SdkBridge.hpp"
#include "module/Simulation/src/Simulation.hpp"
#include "module/Supervisor/src/Supervisor.hpp"
#include "module/Viewer/src/Viewer.hpp"
#include "shared/CliOptions.hpp"
#include "shared/gl/XThreads.hpp"

namespace {
    void handle_signal(int /*signum*/) {
        if (NUClear::PowerPlant::powerplant != nullptr) {
            NUClear::PowerPlant::powerplant->shutdown();
        }
    }
}  // namespace

int main(int argc, char** argv) {
    k1sim::init_x_threads();  // must precede any GL/X11 use (Viewer + Camera render threads)
    k1sim::cli() = k1sim::parse_cli(argc, argv);

    NUClear::Configuration config;
    config.default_pool_concurrency = 4;
    NUClear::PowerPlant plant(config);

    // NUClear does not auto-install the chrono extension; without it no Every<>
    // reaction (viewer tick, battery publisher) ever fires.
    plant.install<NUClear::extension::ChronoController>();
    plant.install<NUClear::extension::IOController>();
    plant.install<k1sim::module::ConsoleLog>();
    plant.install<k1sim::module::Simulation>();
    plant.install<k1sim::module::Locomotion>();
    plant.install<k1sim::module::SdkBridge>();
    plant.install<k1sim::module::Supervisor>();
    plant.install<k1sim::module::Viewer>();

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Blocks; MainThread reactions (the viewer) run on this thread.
    plant.start();
    return 0;
}
