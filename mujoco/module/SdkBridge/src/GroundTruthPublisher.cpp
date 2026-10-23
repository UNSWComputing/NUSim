#include "module/SdkBridge/src/GroundTruthPublisher.hpp"

#include <array>
#include <cstdint>

#include "Odometry.h"
#include "OdometryPubSubTypes.h"
#include "shared/k1/NUSimApi.hpp"

namespace k1sim::module::sdkbridge {

    namespace {

        // Pose and twist both in the world frame. child_frame_id repeats "world" to say so: the
        // ROS convention would put the twist in the child frame, which is not what this carries.
        nav_msgs::msg::dds_::Odometry_ make_odometry(int64_t wall_time_ns,
                                                     const std::array<double, 3>& position,
                                                     const std::array<double, 4>& quat_wxyz,
                                                     const std::array<double, 3>& lin_vel,
                                                     const std::array<double, 3>& ang_vel) {
            nav_msgs::msg::dds_::Odometry_ msg;
            msg.header().stamp().sec(static_cast<int32_t>(wall_time_ns / 1000000000LL));
            msg.header().stamp().nanosec(static_cast<uint32_t>(wall_time_ns % 1000000000LL));
            msg.header().frame_id("world");
            msg.child_frame_id("world");
            msg.pose().pose().position().x(position[0]);
            msg.pose().pose().position().y(position[1]);
            msg.pose().pose().position().z(position[2]);
            msg.pose().pose().orientation().w(quat_wxyz[0]);
            msg.pose().pose().orientation().x(quat_wxyz[1]);
            msg.pose().pose().orientation().y(quat_wxyz[2]);
            msg.pose().pose().orientation().z(quat_wxyz[3]);
            msg.twist().twist().linear().x(lin_vel[0]);
            msg.twist().twist().linear().y(lin_vel[1]);
            msg.twist().twist().linear().z(lin_vel[2]);
            msg.twist().twist().angular().x(ang_vel[0]);
            msg.twist().twist().angular().y(ang_vel[1]);
            msg.twist().twist().angular().z(ang_vel[2]);
            return msg;
        }

    }  // namespace

    GroundTruthPublisher::GroundTruthPublisher(DdsParticipant& dds) {
        using nav_msgs::msg::dds_::Odometry_PubSubType;
        ball_writer_  = dds.create_writer<Odometry_PubSubType>(k1sim::nusim::TOPIC_GT_BALL,
                                                              DdsParticipant::state_writer_qos());
        robot_writer_ = dds.create_writer<Odometry_PubSubType>(k1sim::nusim::TOPIC_GT_ROBOT,
                                                               DdsParticipant::state_writer_qos());
    }

    void GroundTruthPublisher::publish(const k1sim::message::SimStateUpdate& update) {
        const auto& base = update.base;
        auto robot       = make_odometry(update.wall_time_ns,
                                   {base.x, base.y, base.z},
                                   base.quat,
                                   base.lin_vel,
                                   base.ang_vel);
        robot_writer_->write(&robot);

        if (update.ball.valid) {
            // The ball's orientation carries no information (it is a uniform sphere), so it is left
            // as identity rather than tracking the free body's accumulated spin.
            auto ball = make_odometry(update.wall_time_ns,
                                      update.ball.position,
                                      {1.0, 0.0, 0.0, 0.0},
                                      update.ball.lin_vel,
                                      update.ball.ang_vel);
            ball_writer_->write(&ball);
        }
    }

}  // namespace k1sim::module::sdkbridge
