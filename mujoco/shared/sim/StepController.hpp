#ifndef K1SIM_SHARED_SIM_STEPCONTROLLER_HPP
#define K1SIM_SHARED_SIM_STEPCONTROLLER_HPP

#include <mujoco/mujoco.h>

namespace k1sim {

    // The seam between the physics loop (module::Simulation) and the control logic
    // (module::Locomotion). The physics thread calls step() every physics step with
    // the sim mutex held, immediately before mj_step(); the implementation writes
    // d->ctrl (and, for the kinematic backend, the root free-joint qvel).
    // The state accessors are called from other threads and must be lock-free
    // (atomics) — they feed GetMode replies and the rt/fall_down publisher.
    class StepController {
    public:
        virtual ~StepController() = default;

        // Called under the physics lock. Concrete controllers clear their own mailbox/FSM.
        virtual void reset() {}

        virtual void step(const mjModel* m, mjData* d) = 0;

        virtual int mode() const        = 0;  // booster::RobotMode value
        virtual int fall_state() const  = 0;  // booster::FallState value
        virtual bool getting_up() const = 0;
    };

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_STEPCONTROLLER_HPP
