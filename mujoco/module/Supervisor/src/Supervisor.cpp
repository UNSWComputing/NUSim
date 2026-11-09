#include "module/Supervisor/src/Supervisor.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorConfig.hpp"
#include "shared/CliOptions.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

    namespace gc = k1sim::module::supervisor::gc;
    using k1sim::module::supervisor::RobotConfig;
    using k1sim::module::supervisor::SupervisorConfig;
    using k1sim::module::supervisor::SupervisorLogic;

    namespace {

        constexpr double STAND_Z = 0.555;  // base height of the "ready" pose (as in config/supervisor.yaml)

        // The robots of a --game <n> match: both teams, with the spot each one starts from as its home and a
        // spot outside the touchline to serve penalties on. Robot k's root body is "Trunk" for the main robot
        // and "subNN_Trunk" for the others; the first n robots are the first team in the GameController packet.
        std::vector<RobotConfig> game_robots(int n, bool on_field_positions) {
            const YAML::Node sim_cfg = config::load("simulation.yaml");
            const config::Game game  = config::game(sim_cfg, n, on_field_positions);
            const double width       = sim_cfg["game"]["width"].as<double>();

            std::vector<RobotConfig> robots;
            for (int k = 0; k < static_cast<int>(game.spawns.size()); ++k) {
                const int side  = k < n ? 0 : 1;
                const int index = k % n;
                char body[32];
                if (k == 0) {
                    std::snprintf(body, sizeof(body), "Trunk");
                }
                else {
                    std::snprintf(body, sizeof(body), "sub%02d_Trunk", k);
                }

                RobotConfig rc;
                rc.body       = body;
                rc.team_id    = -1;
                rc.team_index = side;
                rc.player_id  = index + 1;
                rc.home_pose  = {game.spawns[k][0], game.spawns[k][1], STAND_Z, game.spawns[k][2]};

                // Penalised robots wait outside the touchline of their own half, facing the field: the first
                // team behind -y, the second behind +y, spaced out from the halfway line.
                const double sign = side == 0 ? -1.0 : 1.0;
                rc.penalty_pose   = {sign * (1.0 + 0.7 * index), sign * (width / 2.0 + 0.8), STAND_Z, -sign * M_PI / 2.0};
                robots.push_back(rc);
            }
            return robots;
        }

        SupervisorConfig build_config() {
            SupervisorConfig cfg = k1sim::module::supervisor::load_config(config::load("supervisor.yaml"));
            if (cli().game > 0) {
                cfg.robots = game_robots(cli().game, cli().on_field_positions);
            }
            return cfg;
        }

        // Team number -> name from the GameController's teams.yaml. Missing or unreadable: empty.
        std::map<int, std::string> load_team_names(std::string path) {
            std::map<int, std::string> names;
            if (path.empty()) {
                // the GameController's config directory, or the one in its source checkout
                if (const char* dir = std::getenv("GAMECONTROLLER_CONFIG")) {
                    path = std::string(dir) + "/teams.yaml";
                }
                else {
                    std::string checkout;
                    try {
                        checkout = config::load("referee.yaml")["gamecontroller_dir"].as<std::string>("");
                    }
                    catch (const std::exception&) {
                    }
                    if (const char* dir = checkout.empty() ? std::getenv("GAMECONTROLLER_DIR") : nullptr) {
                        checkout = dir;
                    }
                    if (checkout.empty()) {
                        return names;
                    }
                    path = checkout + "/config/teams.yaml";
                }
            }
            try {
                for (const auto& team : YAML::LoadFile(path)) {
                    names[team["number"].as<int>()] = team["name"].as<std::string>();
                }
            }
            catch (const std::exception&) {
                names.clear();
            }
            return names;
        }

        // A UDP socket on the GameController port that other listeners on this host can share.
        int open_listener(int port, std::string& error) {
            const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (fd < 0) {
                error = std::strerror(errno);
                return -1;
            }
            const int yes = 1;
            ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
            ::setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif
            ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
            const timeval timeout{0, 300000};  // wake up regularly to notice shutdown
            ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            sockaddr_in address{};
            address.sin_family      = AF_INET;
            address.sin_port        = htons(static_cast<uint16_t>(port));
            address.sin_addr.s_addr = htonl(INADDR_ANY);
            if (::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
                error = std::strerror(errno);
                ::close(fd);
                return -1;
            }
            return fd;
        }

    }  // namespace

    Supervisor::Supervisor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Loaded eagerly (constructor body, not Startup) so everything is known before the listener starts.
        SupervisorConfig cfg = build_config();
        const bool enabled   = cfg.enabled || cli().gamecontroller || cli().referee;
        const int gc_port    = cfg.gc_port;
        team_names_          = load_team_names(cfg.teams_file);
        logic_               = std::make_unique<SupervisorLogic>(std::move(cfg));

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            model_.store(handles.model, std::memory_order_release);
            data_.store(handles.data, std::memory_order_release);
            sim_mutex_.store(handles.mutex, std::memory_order_release);
        });

        if (!enabled) {
            on<Startup>().then([this] {
                log<NUClear::LogLevel::INFO>("Supervisor disabled (start with --gamecontroller, or set supervisor.yaml enabled: true)");
            });
            on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
            return;
        }

        // The packets are handled in a reaction (not on the listener thread), one at a time.
        on<Trigger<gc::GameControllerPacket>, Single>().then(
            [this](const gc::GameControllerPacket& packet) { handle_packet(packet); });

        // Binding can fail if another program holds the port without sharing it. That must not crash the
        // whole sim: degrade to "no supervisor" and say so.
        // With the built-in referee (--referee) the packets come from module::Referee in-process; listening on
        // the network as well would only deliver each of them a second time.
        if (cli().referee) {
            on<Startup>().then([this] {
                log<NUClear::LogLevel::INFO>("Supervisor ready - following the built-in referee with", logic_->config().robots.size(),
                                             "robot(s) under its control");
            });
            on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
            return;
        }

        std::string error;
        socket_fd_ = open_listener(gc_port, error);

        on<Startup>().then([this, gc_port, error] {
            if (socket_fd_ < 0) {
                log<NUClear::LogLevel::WARN>("Supervisor: could not listen on UDP port", gc_port, "(", error,
                                             ") — running without the GameController");
                return;
            }
            listening_.store(true);
            listener_ = std::thread([this] { listen(socket_fd_); });
            log<NUClear::LogLevel::INFO>("Supervisor ready — waiting for the GameController on UDP port", gc_port, "with",
                                         logic_->config().robots.size(), "robot(s) under its control");
        });

        on<Shutdown>().then([this] {
            listening_.store(false);
            if (listener_.joinable()) {
                listener_.join();
            }
            if (socket_fd_ >= 0) {
                ::close(socket_fd_);
                socket_fd_ = -1;
            }
            log<NUClear::LogLevel::INFO>("Supervisor shutting down");
        });
    }

    void Supervisor::listen(int fd) {
        std::uint8_t buffer[2048];
        while (listening_.load()) {
            const ssize_t received = ::recv(fd, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                continue;  // timeout (checks listening_ again) or a transient error
            }
            // Anything that is not a GameController packet of the version we speak is ignored silently:
            // unrelated traffic on the port is expected, not an error.
            auto packet = std::make_unique<gc::GameControllerPacket>();
            if (gc::try_parse(buffer, static_cast<std::size_t>(received), *packet)) {
                emit(packet);
            }
        }
    }

    void Supervisor::handle_packet(const gc::GameControllerPacket& parsed) {
        // Publish the referee state for the browser viewer (module::WebViewer).
        auto team_name = [this](const gc::Team& team) {
            const auto found = team_names_.find(team.team_id);
            return found != team_names_.end() ? found->second : "Team " + std::to_string(team.team_id);
        };
        auto game            = std::make_unique<message::GameState>();
        game->left           = team_name(parsed.teams[0]);
        game->right          = team_name(parsed.teams[1]);
        game->score_left     = parsed.teams[0].score;
        game->score_right    = parsed.teams[1].score;
        game->secs_remaining = parsed.secs_remaining;
        game->state          = gc::state_name(parsed.state);
        game->first_half     = parsed.first_half;
        game->secondary_time = parsed.secondary_time;
        game->colour_left    = static_cast<int>(parsed.teams[0].field_player_colour);
        game->colour_right   = static_cast<int>(parsed.teams[1].field_player_colour);
        game->kicking_side   = parsed.kicking_team == parsed.teams[0].team_id   ? 0
                               : parsed.kicking_team == parsed.teams[1].team_id ? 1
                                                                                : -1;
        game->mode = parsed.game_phase == gc::GamePhase::PENALTY_SHOOTOUT ? "penalty shootout"
                     : parsed.game_phase == gc::GamePhase::TIMEOUT        ? "timeout"
                     : parsed.first_half                                  ? "first half"
                                                                          : "second half";
        game->mode += std::string(" | ") + gc::state_name(parsed.state);
        for (const auto& rc : logic_->config().robots) {
            // Same team lookup as SupervisorLogic: by team number if configured, else by position in the packet.
            int side = rc.team_index;
            if (rc.team_id >= 0) {
                side = parsed.teams[0].team_id == rc.team_id ? 0 : parsed.teams[1].team_id == rc.team_id ? 1 : -1;
            }
            if (side < 0 || side > 1 || rc.player_id < 1 || static_cast<std::size_t>(rc.player_id) > gc::MAX_NUM_PLAYERS) {
                continue;
            }
            const gc::Robot& info = parsed.teams[static_cast<std::size_t>(side)].players[static_cast<std::size_t>(rc.player_id) - 1];
            message::GameState::Robot robot;
            robot.body   = rc.body;
            robot.side   = side;
            robot.player = rc.player_id;
            if (info.penalty_state != gc::PenaltyState::UNPENALISED) {
                robot.penalty      = gc::penalty_state_name(info.penalty_state);
                robot.penalty_secs = info.penalised_time_left;
            }
            game->robots.push_back(robot);
        }
        emit(game);

        const mjModel* m      = model_.load(std::memory_order_acquire);
        mjData* d             = data_.load(std::memory_order_acquire);
        std::mutex* sim_mutex = sim_mutex_.load(std::memory_order_acquire);
        if (m == nullptr || d == nullptr || sim_mutex == nullptr) {
            log<NUClear::LogLevel::WARN>("GameController packet received before the sim model was ready — ignoring");
            return;
        }

        std::vector<SupervisorLogic::Action> actions;
        {
            std::lock_guard<std::mutex> lock(*sim_mutex);
            actions = logic_->process(m, d, parsed);
        }
        for (const auto& action : actions) {
            if (action.level == SupervisorLogic::Action::Level::WARN) {
                log<NUClear::LogLevel::WARN>(action.message.c_str());
            }
            else {
                log<NUClear::LogLevel::INFO>(action.message.c_str());
            }
        }
    }

}  // namespace k1sim::module
