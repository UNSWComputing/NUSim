#ifndef K1SIM_MODULE_SDKBRIDGE_GROUNDTRUTHPUBLISHER_HPP
#define K1SIM_MODULE_SDKBRIDGE_GROUNDTRUTHPUBLISHER_HPP

#include <fastdds/dds/publisher/DataWriter.hpp>

#include "module/SdkBridge/src/DdsParticipant.hpp"
#include "shared/message/SimMessages.hpp"

// Publishes NUSim ground truth for validating the NUbots estimators against the simulator
// (rt/nusim/gt/ball and rt/nusim/gt/robot, see shared/k1/NUSimApi.hpp and PROTOCOL.md §6).
// Not part of the Booster SDK surface: a real robot has no such topics.

namespace k1sim::module::sdkbridge {

    class GroundTruthPublisher {
    public:
        explicit GroundTruthPublisher(DdsParticipant& dds);

        // Called at the state cadence (50 Hz) with each physics snapshot. The ball topic is only
        // written while the scene has a ball.
        void publish(const k1sim::message::SimStateUpdate& update);

    private:
        eprosima::fastdds::dds::DataWriter* ball_writer_  = nullptr;
        eprosima::fastdds::dds::DataWriter* robot_writer_ = nullptr;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_GROUNDTRUTHPUBLISHER_HPP
