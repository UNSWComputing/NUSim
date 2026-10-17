#include "module/SdkBridge/src/DdsParticipant.hpp"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/rtps/attributes/BuiltinTransports.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <memory>
#include <stdexcept>

namespace k1sim::module::sdkbridge {

    using namespace eprosima::fastdds::dds;  // NOLINT — matches Fast-DDS's own usage idiom

    DdsParticipant::DdsParticipant(int domain_id, bool udp_only, bool local_only) {
        DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
        pqos.name("k1sim_sdkbridge");
        if (local_only) {
            auto transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
            transport->interfaceWhiteList.emplace_back("127.0.0.1");
            transport->non_blocking_send            = true;
            pqos.transport().use_builtin_transports = false;
            pqos.transport().user_transports        = {transport};
        }
        else {
            pqos.setup_transports(udp_only ? eprosima::fastdds::rtps::BuiltinTransports::UDPv4
                                           : eprosima::fastdds::rtps::BuiltinTransports::DEFAULT);
        }

        participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, pqos);
        if (participant_ == nullptr) {
            throw std::runtime_error("DdsParticipant: failed to create DomainParticipant on domain "
                                     + std::to_string(domain_id));
        }

        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT);
        if (publisher_ == nullptr) {
            throw std::runtime_error("DdsParticipant: failed to create Publisher");
        }

        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (subscriber_ == nullptr) {
            throw std::runtime_error("DdsParticipant: failed to create Subscriber");
        }
    }

    DdsParticipant::~DdsParticipant() {
        if (participant_ != nullptr) {
            participant_->delete_contained_entities();
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }
    }

    DataWriterQos DdsParticipant::state_writer_qos(int depth) {
        DataWriterQos qos      = DATAWRITER_QOS_DEFAULT;
        qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        qos.durability().kind  = VOLATILE_DURABILITY_QOS;
        qos.history().kind     = KEEP_LAST_HISTORY_QOS;
        qos.history().depth    = depth;
        return qos;
    }

    DataReaderQos DdsParticipant::rpc_request_reader_qos(int depth) {
        DataReaderQos qos      = DATAREADER_QOS_DEFAULT;
        qos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        qos.history().kind     = KEEP_LAST_HISTORY_QOS;
        qos.history().depth    = depth;
        return qos;
    }

    Topic* DdsParticipant::get_or_create_topic(const std::string& topic_name, TypeSupport& type) {
        auto it = topics_.find(topic_name);
        if (it != topics_.end()) {
            return it->second;
        }
        type.register_type(participant_);
        Topic* topic = participant_->create_topic(topic_name, type.get_type_name(), TOPIC_QOS_DEFAULT);
        if (topic == nullptr) {
            throw std::runtime_error("DdsParticipant: failed to create topic " + topic_name);
        }
        topics_[topic_name] = topic;
        return topic;
    }

}  // namespace k1sim::module::sdkbridge
