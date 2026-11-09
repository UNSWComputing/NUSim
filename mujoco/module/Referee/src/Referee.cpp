#include "module/Referee/src/Referee.hpp"

#include <arpa/inet.h>
#include <array>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "shared/CliOptions.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

    namespace gc = k1sim::module::supervisor::gc;

    namespace {

        // Mirrors of the C types in the GameController's GameController.h (plain C enums and structs).
        enum Side : int { HOME = 0, AWAY = 1 };
        enum SideMapping : int { HOME_DEFENDS_LEFT_GOAL = 0, HOME_DEFENDS_RIGHT_GOAL = 1 };
        enum ActionSource : int { SOURCE_NETWORK = 0, SOURCE_TIMER = 1, SOURCE_USER = 2 };
        enum SetPlay : int { NO_SET_PLAY, KICK_OFF, DIRECT_FREE_KICK, INDIRECT_FREE_KICK, PENALTY_KICK, THROW_IN, GOAL_KICK, CORNER_KICK };
        struct TeamParams {
            std::uint8_t number;
            int field_player_colour;
            int goalkeeper_colour;
        };
        struct TestParams {
            bool no_delay;
            bool penalty_shootout;
        };

        const std::array<const char*, 10> COLOURS = {"blue", "red", "yellow", "black", "white", "green", "orange", "purple", "brown", "gray"};
        const std::array<const char*, 12> PENALTY_CALLS = {"illegal_position", "motion_in_set", "motion_in_stop", "local_game_stuck",
                                                           "incapable_robot", "request_for_pick_up", "ball_holding", "leaving_the_field",
                                                           "playing_with_arms_hands", "pushing", "caution", "send_off"};
        const std::map<std::string, SetPlay> SET_PLAYS = {{"direct_free_kick", DIRECT_FREE_KICK}, {"indirect_free_kick", INDIRECT_FREE_KICK},
                                                          {"penalty_kick", PENALTY_KICK},         {"throw_in", THROW_IN},
                                                          {"goal_kick", GOAL_KICK},               {"corner_kick", CORNER_KICK}};

        template <typename Names>
        int index_of(const Names& names, const std::string& name) {
            for (std::size_t i = 0; i < names.size(); ++i) {
                if (name == names[i]) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        }

        TeamParams load_team(const YAML::Node& node, std::uint8_t number, const char* colour, const char* keeper) {
            TeamParams team{};
            team.number              = static_cast<std::uint8_t>(node["number"].as<int>(number));
            team.field_player_colour = index_of(COLOURS, node["field_player_colour"].as<std::string>(colour));
            team.goalkeeper_colour   = index_of(COLOURS, node["goalkeeper_colour"].as<std::string>(keeper));
            if (team.field_player_colour < 0 || team.goalkeeper_colour < 0) {
                throw std::runtime_error("config/referee.yaml: unknown jersey colour");
            }
            return team;
        }

    }  // namespace

    struct Referee::Library {
        void* handle = nullptr;

        void* (*competition_params_new)(const std::uint8_t*, std::uintptr_t)                              = nullptr;
        void (*competition_params_destroy)(void*)                                                         = nullptr;
        void* (*params_new)(const void*, const TeamParams*, const TeamParams*, Side, SideMapping, TestParams) = nullptr;
        void (*params_destroy)(void*)                                                                     = nullptr;
        void* (*create)(const void*)                                                                      = nullptr;
        void (*destroy)(void*)                                                                            = nullptr;
        void (*seek)(void*, std::uint64_t)                                                                = nullptr;
        bool (*apply)(void*, void*, ActionSource)                                                         = nullptr;
        bool (*is_legal)(void*, const void*)                                                              = nullptr;
        void (*action_destroy)(void*)                                                                     = nullptr;
        bool (*read)(void*, std::uint8_t, bool, std::uint8_t*, std::uintptr_t)                            = nullptr;
        const char* (*last_error)()                                                                       = nullptr;

        void* (*finish_half)()                             = nullptr;
        void* (*finish_set_play)()                         = nullptr;
        void* (*free_set_play)()                           = nullptr;
        void* (*global_game_stuck)()                       = nullptr;
        void* (*goal)(Side)                                = nullptr;
        void* (*penalize)(Side, std::uint8_t, int)         = nullptr;
        void* (*start_set_play)(const Side*, SetPlay)      = nullptr;
        void* (*stop_play)(bool)                           = nullptr;
        void* (*switch_half)()                             = nullptr;
        void* (*timeout)(const Side*)                      = nullptr;
        void* (*undo)(std::uint32_t)                       = nullptr;
        void* (*unpenalize)(Side, std::uint8_t, bool)      = nullptr;
        void* (*wait_for_set_play)()                       = nullptr;

        template <typename Function>
        bool bind(Function& function, const char* name, std::string& error) {
            function = reinterpret_cast<Function>(::dlsym(handle, name));
            if (function == nullptr) {
                error = std::string("the GameController library has no function ") + name;
            }
            return function != nullptr;
        }

        bool load(const std::string& path, std::string& error) {
            handle = ::dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (handle == nullptr) {
                error = std::string("cannot load ") + path + " (" + ::dlerror() + ")";
                return false;
            }
            return bind(competition_params_new, "gc_competition_params_new", error)
                   && bind(competition_params_destroy, "gc_competition_params_destroy", error)
                   && bind(params_new, "gc_params_new", error) && bind(params_destroy, "gc_params_destroy", error)
                   && bind(create, "gc_new", error) && bind(destroy, "gc_destroy", error) && bind(seek, "gc_seek", error)
                   && bind(apply, "gc_apply", error) && bind(is_legal, "gc_is_legal", error)
                   && bind(action_destroy, "gc_action_destroy", error) && bind(read, "gc_read", error)
                   && bind(last_error, "gc_last_error", error) && bind(finish_half, "gc_action_finish_half", error)
                   && bind(finish_set_play, "gc_action_finish_set_play", error)
                   && bind(free_set_play, "gc_action_free_set_play", error)
                   && bind(global_game_stuck, "gc_action_global_game_stuck", error) && bind(goal, "gc_action_goal", error)
                   && bind(penalize, "gc_action_penalize", error) && bind(start_set_play, "gc_action_start_set_play", error)
                   && bind(stop_play, "gc_action_stop_play", error) && bind(switch_half, "gc_action_switch_half", error)
                   && bind(timeout, "gc_action_timeout", error) && bind(undo, "gc_action_undo", error)
                   && bind(unpenalize, "gc_action_unpenalize", error)
                   && bind(wait_for_set_play, "gc_action_wait_for_set_play", error);
        }

        // The action a command stands for, or nullptr if the command is unknown or incomplete.
        void* action(const message::RefereeCommand& c) const {
            const Side side       = c.side == "away" ? AWAY : HOME;
            const bool has_side   = c.side == "home" || c.side == "away";
            const auto player     = static_cast<std::uint8_t>(c.player);
            if (c.action == "kickoff" && has_side) return start_set_play(&side, KICK_OFF);
            if (c.action == "setplay" && has_side && SET_PLAYS.count(c.arg) != 0) return start_set_play(&side, SET_PLAYS.at(c.arg));
            if (c.action == "set") return wait_for_set_play();
            if (c.action == "play") return free_set_play();
            if (c.action == "finish_set_play") return finish_set_play();
            if (c.action == "stop") return stop_play(false);
            if (c.action == "resume") return stop_play(true);
            if (c.action == "goal" && has_side) return goal(side);
            if (c.action == "finish_half") return finish_half();
            if (c.action == "switch_half") return switch_half();
            if (c.action == "timeout") return timeout(has_side ? &side : nullptr);
            if (c.action == "stuck") return global_game_stuck();
            if (c.action == "undo") return undo(1);
            if (c.action == "penalize" && has_side && index_of(PENALTY_CALLS, c.arg) >= 0) {
                return penalize(side, player, index_of(PENALTY_CALLS, c.arg));
            }
            if (c.action == "unpenalize" && has_side) return unpenalize(side, player, c.arg == "force");
            return nullptr;
        }

        ~Library() {
            if (handle != nullptr) {
                ::dlclose(handle);
            }
        }
    };

    Referee::Referee(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        if (!cli().referee) {
            return;
        }

        std::string error;
        if (!start(error)) {
            on<Startup>().then([this, error] { log<NUClear::LogLevel::ERROR>("Referee disabled:", error); });
            return;
        }

        on<Startup>().then([this] {
            log<NUClear::LogLevel::INFO>("Referee ready - the built-in GameController announces the game on UDP port",
                                         ntohs(targets_.empty() ? 0 : targets_.front().sin_port));
        });

        // Game time follows simulation time, so a slowed down or free-running sim still plays a full half.
        on<Trigger<message::SimStateUpdate>, Single>().then(
            [this](const message::SimStateUpdate& state) { advance(state.sim_time); });

        on<Trigger<message::RefereeCommand>>().then([this](const message::RefereeCommand& command) { apply(command); });
    }

    Referee::~Referee() {
        if (game_ != nullptr && lib_) {
            lib_->destroy(game_);
        }
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
        }
    }

    bool Referee::start(std::string& error) {
        const YAML::Node cfg = config::load("referee.yaml");

        std::string dir = cfg["gamecontroller_dir"].as<std::string>("");
        if (dir.empty()) {
            const char* env = std::getenv("GAMECONTROLLER_DIR");
            dir             = env != nullptr ? env : "";
        }
        if (dir.empty()) {
            error = "set gamecontroller_dir in config/referee.yaml (or GAMECONTROLLER_DIR) to the GameController checkout";
            return false;
        }
        const std::filesystem::path root(dir);

#ifdef __APPLE__
        const char* library_name = "libgame_controller_api.dylib";
#else
        const char* library_name = "libgame_controller_api.so";
#endif
        const std::filesystem::path library = root / "target" / "release" / library_name;
        if (!std::filesystem::exists(library)) {
            error = library.string() + " not found - build it with: cargo build -r -p game_controller_api";
            return false;
        }
        lib_ = std::make_unique<Library>();
        if (!lib_->load(library.string(), error)) {
            return false;
        }

        const std::string competition = cfg["competition"].as<std::string>("middle_advanced");
        std::ifstream file(root / "config" / competition / "params.yaml");
        if (!file.is_open()) {
            error = "no params.yaml for competition '" + competition + "' in " + (root / "config").string();
            return false;
        }
        std::ostringstream yaml;
        yaml << file.rdbuf();
        const std::string text = yaml.str();
        void* competition_params =
            lib_->competition_params_new(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
        if (competition_params == nullptr) {
            error = std::string("GameController: ") + lib_->last_error();
            return false;
        }

        const TeamParams home = load_team(cfg["home"], 1, "blue", "white");
        const TeamParams away = load_team(cfg["away"], 2, "red", "yellow");
        const Side kick_off   = cfg["kick_off"].as<std::string>("home") == "away" ? AWAY : HOME;
        const TestParams test{cfg["no_delay"].as<bool>(false), false};
        // The first --game team (home) stands in the -x half, i.e. it defends the left goal.
        void* params = lib_->params_new(competition_params, &home, &away, kick_off, HOME_DEFENDS_LEFT_GOAL, test);
        lib_->competition_params_destroy(competition_params);
        game_ = params != nullptr ? lib_->create(params) : nullptr;
        if (params != nullptr) {
            lib_->params_destroy(params);
        }
        if (game_ == nullptr) {
            error = std::string("GameController: ") + lib_->last_error();
            return false;
        }

        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ >= 0) {
            const int yes = 1;
            ::setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
        }
        const int port = cfg["port"].as<int>(3838);
        for (const auto& target : cfg["targets"]) {
            sockaddr_in address{};
            address.sin_family = AF_INET;
            address.sin_port   = htons(static_cast<std::uint16_t>(port));
            if (::inet_pton(AF_INET, target.as<std::string>().c_str(), &address.sin_addr) == 1) {
                targets_.push_back(address);
            }
        }
        return true;
    }

    void Referee::advance(double sim_time) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_sim_time_ < 0.0 || sim_time < last_sim_time_) {
            last_sim_time_ = sim_time;  // first update, or the sim was reset
            publish();
            return;
        }
        const double elapsed = sim_time - last_sim_time_;
        last_sim_time_       = sim_time;

        // The GameController counts whole milliseconds; carry the fractions.
        pending_ms_ += elapsed * 1000.0;
        const auto whole = static_cast<std::uint64_t>(pending_ms_);
        if (whole > 0) {
            lib_->seek(game_, whole);
            pending_ms_ -= static_cast<double>(whole);
        }

        since_publish_s_ += elapsed;
        if (since_publish_s_ >= 0.5) {
            publish();
        }
    }

    void Referee::apply(const message::RefereeCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        void* action = lib_->action(command);
        ++result_seq_;
        if (action == nullptr) {
            result_ok_   = false;
            result_text_ = "unknown referee command: " + command.action;
        }
        else {
            result_ok_   = lib_->apply(game_, action, SOURCE_USER);  // takes the action, applied or not
            result_text_ = command.action + (result_ok_ ? "" : " is not allowed right now");
        }
        log<NUClear::LogLevel::INFO>("referee:", command.action, command.side, command.player > 0 ? std::to_string(command.player) : "",
                                     command.arg, result_ok_ ? "" : "(not allowed)");
        publish();
    }

    void Referee::publish() {
        since_publish_s_ = 0.0;

        // What the robots are told (possibly delayed after kick-off, like the real GameController) goes out on
        // the network; the true state goes to the Supervisor, which places ball and robots.
        std::array<std::uint8_t, sizeof(gc::GameControllerPacket)> buffer{};
        if (lib_->read(game_, packet_number_++, false, buffer.data(), buffer.size()) && socket_fd_ >= 0) {
            for (const auto& target : targets_) {
                ::sendto(socket_fd_, buffer.data(), buffer.size(), 0, reinterpret_cast<const sockaddr*>(&target), sizeof(target));
            }
        }
        if (lib_->read(game_, packet_number_, true, buffer.data(), buffer.size())) {
            // True data carries the monitor magic ("RGTD"); the Supervisor expects the players' one.
            std::memcpy(buffer.data(), gc::RECEIVE_HEADER.data(), gc::RECEIVE_HEADER.size());
            auto packet = std::make_unique<gc::GameControllerPacket>();
            if (gc::try_parse(buffer.data(), buffer.size(), *packet)) {
                emit(packet);
            }
        }

        // Which buttons the referee panel may offer right now.
        auto status = std::make_unique<message::RefereeStatus>();
        auto offer  = [this, &status](const std::string& name, const message::RefereeCommand& command) {
            void* action = lib_->action(command);
            if (action != nullptr) {
                if (lib_->is_legal(game_, action)) {
                    status->legal.push_back(name);
                }
                lib_->action_destroy(action);
            }
        };
        for (const char* side : {"home", "away"}) {
            offer(std::string("kickoff:") + side, {"kickoff", side, 0, ""});
            offer(std::string("goal:") + side, {"goal", side, 0, ""});
            offer(std::string("timeout:") + side, {"timeout", side, 0, ""});
            offer(std::string("setplay:") + side, {"setplay", side, 0, "direct_free_kick"});
        }
        for (const char* name : {"set", "play", "finish_set_play", "stop", "resume", "finish_half", "switch_half", "stuck", "undo"}) {
            offer(name, {name, "", 0, ""});
        }
        offer("timeout:none", {"timeout", "", 0, ""});
        status->result_seq  = result_seq_;
        status->result_ok   = result_ok_;
        status->result_text = result_text_;

        std::string key = std::to_string(result_seq_);
        for (const auto& name : status->legal) {
            key += "|" + name;
        }
        if (key != last_status_) {
            last_status_ = key;
            emit(status);
        }
    }

}  // namespace k1sim::module
