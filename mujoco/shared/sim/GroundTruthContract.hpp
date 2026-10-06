#ifndef K1SIM_SHARED_SIM_GROUNDTRUTHCONTRACT_HPP
#define K1SIM_SHARED_SIM_GROUNDTRUTHCONTRACT_HPP

#include <cstdint>

namespace k1sim::ground_truth {

    inline constexpr uint32_t SCHEMA_VERSION          = 1;
    inline constexpr const char* TOPIC_WORLD_SNAPSHOT = "rt/nusim/gt/world_v1";
    inline constexpr const char* TYPE_WORLD_SNAPSHOT  = "nusim_msgs::msg::dds_::WorldSnapshot_";

}  // namespace k1sim::ground_truth

#endif  // K1SIM_SHARED_SIM_GROUNDTRUTHCONTRACT_HPP
