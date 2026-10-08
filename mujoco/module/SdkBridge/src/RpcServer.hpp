#ifndef K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP
#define K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP

#include <atomic>
#include <cstdint>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <nuclear>

#include "module/SdkBridge/src/DdsParticipant.hpp"

// The DDS-facing half of the LocoApi RPC surface: a DataReaderListener on
// rt/LocoApiTopicReq that hands each request's header/body JSON to the pure
// dispatch core (RpcDispatch.hpp/.cpp), emits the resulting NUClear message (if
// any), and replies IMMEDIATELY on rt/LocoApiTopicResp — synchronously inside the
// DDS callback thread, per module/SdkBridge/PROTOCOL.md §3 (B1LocoClient blocks up
// to 1000 ms per call).
//
// Also owns the optional rt/joint_ctrl (LowCmd_) reader: a straightforward
// passthrough that emits LowCmdMessage.

namespace k1sim::module::sdkbridge {

    class RpcServer : public eprosima::fastdds::dds::DataReaderListener {
    public:
        RpcServer(DdsParticipant& dds, NUClear::Reactor& reactor, int64_t unknown_api_status, int robot_id = 1);
        ~RpcServer() override;

        void on_data_available(eprosima::fastdds::dds::DataReader* reader) override;

        // SdkBridge's RobotStatesUpdate handler calls this every tick so
        // GET_MODE can answer from an atomic without touching the physics thread.
        void set_current_mode(int mode) {
            current_mode_.store(mode, std::memory_order_relaxed);
        }

    private:
        void handle_rpc_request(eprosima::fastdds::dds::DataReader* reader);
        void handle_joint_ctrl(eprosima::fastdds::dds::DataReader* reader);

        DdsParticipant& dds_;
        int robot_id_;
        NUClear::Reactor& reactor_;
        int64_t unknown_api_status_;
        std::atomic<int> current_mode_{0};

        eprosima::fastdds::dds::DataReader* rpc_req_reader_    = nullptr;
        eprosima::fastdds::dds::DataReader* joint_ctrl_reader_ = nullptr;
        eprosima::fastdds::dds::DataWriter* rpc_resp_writer_   = nullptr;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_RPCSERVER_HPP
