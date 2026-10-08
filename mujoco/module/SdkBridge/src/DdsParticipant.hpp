#ifndef K1SIM_MODULE_SDKBRIDGE_DDSPARTICIPANT_HPP
#define K1SIM_MODULE_SDKBRIDGE_DDSPARTICIPANT_HPP

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <map>
#include <string>

// Thin wrapper around a single eprosima::fastdds::dds::DomainParticipant, matching
// the SDK's ChannelFactory::Init(domain_id) — one participant, one Publisher, one
// Subscriber, typed writer/reader helpers. See module/SdkBridge/PROTOCOL.md §4 for
// the QoS rationale (state_writer_qos / rpc_request_reader_qos) and the udp_only
// transport fallback.

namespace k1sim::module::sdkbridge {

    class DdsParticipant {
    public:
        // udp_only strips the SHM transport (config/dds.yaml: udp_only, or env
        // K1_DDS_UDP_ONLY=1), leaving UDPv4 only — a workaround for docker boundaries
        // where /dev/shm isn't shared (e.g. no --ipc host).
        DdsParticipant(int domain_id, bool udp_only);
        ~DdsParticipant();

        DdsParticipant(const DdsParticipant&)            = delete;
        DdsParticipant& operator=(const DdsParticipant&) = delete;

        // PROTOCOL.md §4: state writers use RELIABLE + VOLATILE + KEEP_LAST(depth).
        static eprosima::fastdds::dds::DataWriterQos state_writer_qos(int depth = 5);
        // PROTOCOL.md §4: the RPC request reader uses RELIABLE + KEEP_LAST(depth) so a
        // burst of calls at startup (e.g. NUbots' immediate ChangeMode) isn't dropped.
        static eprosima::fastdds::dds::DataReaderQos rpc_request_reader_qos(int depth = 10);

        // PubSubTypeT is a generated `<Type>_PubSubType` (e.g.
        // booster_interface::msg::dds_::Odometer_PubSubType). Registers the type (once
        // per topic_name) and creates the writer/reader on it.
        template <typename PubSubTypeT>
        eprosima::fastdds::dds::DataWriter* create_writer(const std::string& topic_name,
                                                          const eprosima::fastdds::dds::DataWriterQos& qos) {
            eprosima::fastdds::dds::TypeSupport type(new PubSubTypeT());
            auto* topic = get_or_create_topic(topic_name, type);
            return publisher_->create_datawriter(topic, qos);
        }

        template <typename PubSubTypeT>
        eprosima::fastdds::dds::DataReader* create_reader(const std::string& topic_name,
                                                          const eprosima::fastdds::dds::DataReaderQos& qos,
                                                          eprosima::fastdds::dds::DataReaderListener* listener) {
            eprosima::fastdds::dds::TypeSupport type(new PubSubTypeT());
            auto* topic = get_or_create_topic(topic_name, type);
            return subscriber_->create_datareader(topic, qos, listener);
        }

        void delete_reader(eprosima::fastdds::dds::DataReader* reader) {
            if (reader) {
                reader->set_listener(nullptr);
                subscriber_->delete_datareader(reader);
            }
        }

        eprosima::fastdds::dds::DomainParticipant* participant() const {
            return participant_;
        }

    private:
        eprosima::fastdds::dds::Topic* get_or_create_topic(const std::string& topic_name,
                                                           eprosima::fastdds::dds::TypeSupport& type);

        eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
        eprosima::fastdds::dds::Publisher* publisher_           = nullptr;
        eprosima::fastdds::dds::Subscriber* subscriber_         = nullptr;
        std::map<std::string, eprosima::fastdds::dds::Topic*> topics_;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_DDSPARTICIPANT_HPP
