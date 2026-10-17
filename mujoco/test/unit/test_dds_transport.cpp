// Exercise discovery, generated message delivery and participant teardown, which link-only tests cannot verify.
#include <chrono>
#include <cstdio>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastrtps/attributes/LibrarySettingsAttributes.h>
#include <fastrtps/xmlparser/XMLProfileManager.h>
#include <thread>

#include "Time.h"
#include "TimePubSubTypes.h"
#include "module/SdkBridge/src/DdsParticipant.hpp"

int main() {
    using builtin_interfaces::msg::dds_::Time_;
    using builtin_interfaces::msg::dds_::Time_PubSubType;
    using k1sim::module::sdkbridge::DdsParticipant;
    using namespace eprosima::fastdds::dds;

    // Exercise transports rather than the same-process delivery shortcut.
    eprosima::fastrtps::LibrarySettingsAttributes settings;
    settings.intraprocess_delivery = eprosima::fastrtps::INTRAPROCESS_OFF;
    eprosima::fastrtps::xmlparser::XMLProfileManager::library_settings(settings);

    for (int mode = 0; mode < 3; ++mode) {
        const bool udp_only   = mode != 1;
        const bool local_only = mode == 2;
        const char* name      = local_only ? "loopback UDP" : (udp_only ? "UDP" : "UDP+SHM");
        // Keep test traffic away from the simulator's domain 0 and real robots.
        DdsParticipant publisher(81 + mode, udp_only, local_only);
        DdsParticipant subscriber(81 + mode, udp_only, local_only);
        auto writer_qos = DdsParticipant::state_writer_qos();
        writer_qos.data_sharing().off();
        auto reader_qos = DdsParticipant::rpc_request_reader_qos();
        reader_qos.data_sharing().off();
        auto* writer = publisher.create_writer<Time_PubSubType>("k1sim_test/time", writer_qos);
        auto* reader = subscriber.create_reader<Time_PubSubType>("k1sim_test/time", reader_qos, nullptr);
        if (writer == nullptr || reader == nullptr) {
            std::fprintf(stderr, "Could not create test DDS endpoints\n");
            return 1;
        }

        Time_ sent;
        sent.sec(123);
        sent.nanosec(456);
        bool received       = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline && !received) {
            writer->write(&sent);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            Time_ sample;
            SampleInfo info;
            while (reader->take_next_sample(&sample, &info) == ReturnCode_t::RETCODE_OK) {
                if (info.valid_data && sample.sec() == sent.sec() && sample.nanosec() == sent.nanosec()) {
                    received = true;
                }
            }
        }
        if (!received) {
            std::fprintf(stderr, "DDS message delivery timed out (%s)\n", name);
            return 1;
        }
        std::printf("DDS message delivered (%s)\n", name);
    }
    return 0;
}
