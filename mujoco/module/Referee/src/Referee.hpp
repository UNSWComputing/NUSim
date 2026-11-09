#ifndef K1SIM_MODULE_REFEREE_HPP
#define K1SIM_MODULE_REFEREE_HPP

// <nuclear> must come before any socket header: on macOS it selects the IPv6 socket API they declare.
#include <nuclear>

#include <cstdint>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <vector>

#include "shared/message/SimMessages.hpp"

namespace k1sim::module {

    // Built-in referee (--referee): the official GameController running inside the sim.
    //
    // The game state, the clocks and the rules come from the GameController's own library
    // (libgame_controller_api, loaded at startup from its source checkout - see config/referee.yaml), so a game
    // here behaves like one run with the GameController application. Referee decisions arrive as
    // message::RefereeCommand (from the browser viewer's referee panel). Time advances with simulation time.
    //
    // The game is announced the way the GameController does it: RoboCupGameControlData packets on UDP port 3838,
    // twice a second and whenever something happens, so robot software cannot tell the difference. The same
    // state goes to module::Supervisor in-process (as its GameControllerPacket), which places ball and robots.
    class Referee : public NUClear::Reactor {
    public:
        explicit Referee(std::unique_ptr<NUClear::Environment> environment);
        ~Referee() override;

    private:
        struct Library;  // the functions of libgame_controller_api

        bool start(std::string& error);
        void apply(const message::RefereeCommand& command);
        void advance(double sim_time);
        void publish();  // caller holds mutex_

        std::unique_ptr<Library> lib_;
        void* game_ = nullptr;  // RoboCup::GameController

        std::mutex mutex_;  // guards the game and everything below
        double last_sim_time_   = -1.0;
        double pending_ms_      = 0.0;
        double since_publish_s_ = 0.0;
        std::uint8_t packet_number_ = 0;
        int result_seq_         = 0;
        std::string result_text_;
        bool result_ok_ = true;
        std::string last_status_;

        int socket_fd_ = -1;
        std::vector<sockaddr_in> targets_;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_REFEREE_HPP
