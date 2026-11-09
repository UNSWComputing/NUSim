#include "module/WebViewer/src/WebViewer.hpp"

#include <cstdlib>
#include <sstream>
#include <string>

#include "shared/CliOptions.hpp"

namespace k1sim::module {

    WebViewer::WebViewer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // cli() is set in main() before any module is installed.
        if (!cli().web && cli().record.empty()) {
            return;
        }

        web::WebStream::Options options;
        options.host        = cli().web_host;
        options.port        = cli().web ? cli().web_port : 0;
        options.web_root    = std::string(K1SIM_SOURCE_DIR) + "/module/WebViewer/web";
        options.record_path = cli().record;

        auto stream = std::make_unique<web::WebStream>(options);
        std::string error;
        if (!stream->start(error)) {
            log<NUClear::LogLevel::ERROR>("Web viewer disabled:", error);
            return;
        }
        stream_ = std::move(stream);

        if (cli().web) {
            log<NUClear::LogLevel::INFO>("Web viewer at http://" + options.host + ":" + std::to_string(options.port) + "/");
        }
        if (!options.record_path.empty()) {
            log<NUClear::LogLevel::INFO>("Recording to", options.record_path);
        }

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            if (handles.model == nullptr || handles.data == nullptr || handles.mutex == nullptr) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            handles_ = handles;
            stream_->set_model(handles.model);  // the model itself is never modified while the sim runs
        });

        on<Trigger<message::SimStateUpdate>, Single>().then([this] {
            std::lock_guard<std::mutex> lock(mutex_);
            if (handles_.model == nullptr || (updates_++ % 2) != 0) {
                return;
            }
            // Same contract as module::Viewer: hold the sim mutex only for the (sub-millisecond) pose copy.
            std::lock_guard<std::mutex> sim_lock(*handles_.mutex);
            stream_->publish_frame(handles_.model, handles_.data);
        });

        // Only arrives while module::Supervisor is enabled (supervisor.yaml) and a GameController is on the network.
        on<Trigger<message::GameState>>().then([this](const message::GameState& game) {
            using web::WebStream;
            std::ostringstream json;
            json << "{\"type\":\"game\",\"left\":" << WebStream::quote(game.left) << ",\"right\":" << WebStream::quote(game.right)
                 << ",\"scoreLeft\":" << game.score_left << ",\"scoreRight\":" << game.score_right << ",\"time\":"
                 << game.secs_remaining << ",\"mode\":" << WebStream::quote(game.mode) << ",\"state\":"
                 << WebStream::quote(game.state) << ",\"firstHalf\":" << (game.first_half ? "true" : "false")
                 << ",\"secondaryTime\":" << game.secondary_time << ",\"colourLeft\":" << game.colour_left
                 << ",\"colourRight\":" << game.colour_right << ",\"kickingSide\":" << game.kicking_side
                 << ",\"robots\":[";
            for (std::size_t i = 0; i < game.robots.size(); ++i) {
                const auto& robot = game.robots[i];
                json << (i > 0 ? "," : "") << "{\"body\":" << WebStream::quote(robot.body) << ",\"side\":" << robot.side
                     << ",\"player\":" << robot.player << ",\"penalty\":" << WebStream::quote(robot.penalty)
                     << ",\"penaltySecs\":" << robot.penalty_secs << "}";
            }
            json << "]}";
            stream_->set_game(json.str());
        });

        // Built-in referee (--referee): its status goes to the viewers' referee panel, their button presses come
        // back as commands. Query format: action=<name>&side=<home|away>&player=<n>&arg=<text>.
        on<Trigger<message::RefereeStatus>>().then([this](const message::RefereeStatus& status) {
            std::ostringstream json;
            json << "{\"type\":\"referee\",\"legal\":[";
            for (std::size_t i = 0; i < status.legal.size(); ++i) {
                json << (i > 0 ? "," : "") << web::WebStream::quote(status.legal[i]);
            }
            json << "],\"seq\":" << status.result_seq << ",\"ok\":" << (status.result_ok ? "true" : "false")
                 << ",\"text\":" << web::WebStream::quote(status.result_text) << "}";
            stream_->set_referee(json.str());
        });
        if (cli().referee) {
            stream_->set_command_handler([this](const std::string& query) {
                auto command = std::make_unique<message::RefereeCommand>();
                std::istringstream pairs(query);
                std::string pair;
                while (std::getline(pairs, pair, '&')) {
                    const std::size_t equals = pair.find('=');
                    if (equals == std::string::npos) {
                        continue;
                    }
                    const std::string key   = pair.substr(0, equals);
                    const std::string value = pair.substr(equals + 1);
                    if (key == "action") command->action = value;
                    if (key == "side") command->side = value;
                    if (key == "arg") command->arg = value;
                    if (key == "player") command->player = std::atoi(value.c_str());
                }
                emit(command);
            });
        }

        on<Shutdown>().then([this] {
            stream_->stop();
        });
    }

}  // namespace k1sim::module
