#ifndef K1SIM_MODULE_SDKBRIDGE_HPP
#define K1SIM_MODULE_SDKBRIDGE_HPP

#include <memory>
#include <nuclear>

#include "module/SdkBridge/src/BallCommandReader.hpp"
#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "module/SdkBridge/src/GroundTruthPublisher.hpp"
#include "module/SdkBridge/src/RpcServer.hpp"
#include "module/SdkBridge/src/StatePublisher.hpp"

namespace k1sim::module {

    // The Booster SDK compatibility surface: FastDDS publishers for
    // rt/low_state, rt/odometer_state, rt/fall_down, rt/battery_state,
    // rt/button_event and the LocoApi RPC server (rt/LocoApiTopicReq/Resp).
    // See module/SdkBridge/PROTOCOL.md for the full wire contract this implements.
    class SdkBridge : public NUClear::Reactor {
    public:
        explicit SdkBridge(std::unique_ptr<NUClear::Environment> environment);

    private:
        // Constructed in on<Startup> (after config/dds.yaml is read) — see DdsParticipant
        // for why a live participant can't be created before that.
        std::unique_ptr<sdkbridge::DdsParticipant> dds_;
        std::unique_ptr<sdkbridge::StatePublisher> state_publisher_;
        std::unique_ptr<sdkbridge::RpcServer> rpc_server_;
        // NUSim-only test surface: ground truth out, ball commands in (PROTOCOL.md §6)
        std::unique_ptr<sdkbridge::GroundTruthPublisher> ground_truth_;
        std::unique_ptr<sdkbridge::BallCommandReader> ball_command_reader_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SDKBRIDGE_HPP
