#ifndef K1SIM_MODULE_LOCOMOTION_HPP
#define K1SIM_MODULE_LOCOMOTION_HPP

#include <memory>
#include <nuclear>
#include <vector>

#include "module/Locomotion/src/LocomotionController.hpp"

namespace k1sim::module {

    // Reduced mode state machine (Damping/Prepare/Custom). Emits ControllerHandle;
    // consumes HeadCommand/ModeChangeRequest/LowCmdMessage and forwards each into
    // LocomotionController's thread-safe setters. Locomotion *policies* live in
    // NUbots_K1 and reach the sim as LowCmd servo targets; the old high-level RPCs
    // (Move/GetUp/LieDown/VisualKick) are accepted on the wire but ignored with a
    // warning. The FSM logic itself lives in LocomotionController (deliberately
    // NUClear-free, see that header); this reactor only wires it to NUClear and
    // logs incoming requests.
    class Locomotion : public NUClear::Reactor {
    public:
        explicit Locomotion(std::unique_ptr<NUClear::Environment> environment);

    private:
        void warn_once(std::atomic<bool>& flag, const char* rpc);

        LocomotionController* controller_for(int robot_id);
        std::vector<std::shared_ptr<LocomotionController>> controllers_;
        std::atomic<bool> walk_warned_{false};
        std::atomic<bool> getup_warned_{false};
        std::atomic<bool> liedown_warned_{false};
        std::atomic<bool> kick_warned_{false};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_HPP
