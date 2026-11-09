#ifndef K1SIM_SHARED_CLIOPTIONS_HPP
#define K1SIM_SHARED_CLIOPTIONS_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace k1sim {

    struct CliOptions {
        bool headless = false;
        std::string field;                // override for simulation.yaml field (a name under its `fields`)
        int game                = 0;      // robots a side for a match (0 = none); sets the field and robots
        bool on_field_positions = false;  // --game robots in kickoff positions, not on the touchlines
        std::string config_dir;           // override for the config directory
        std::string keyframe;             // override for the startup keyframe (default "ready")
        double rtf = -1.0;                // override real-time factor; <0 = use config (0 = free-run)
        int robots = 1;                   // total K1s on the field; extras are PD-held at "ready"
        bool web   = false;               // serve the browser viewer (module::WebViewer)
        std::string web_host = "127.0.0.1";  // address the browser viewer listens on
        int web_port         = 8080;         // port the browser viewer listens on
        std::string record;               // file to record the match to (empty = no recording)
        bool referee        = false;      // run the built-in GameController (module::Referee), operated from the browser viewer
        bool gamecontroller = false;      // listen to the RoboCup GameController (module::Supervisor), whatever supervisor.yaml says
    };

    inline constexpr int MAX_ROBOTS    = 20;
    inline constexpr int MAX_GAME_SIZE = 11;

    // Set once in main() before the PowerPlant starts; read-only afterwards.
    inline CliOptions& cli() {
        static CliOptions options;
        return options;
    }

    inline CliOptions parse_cli(int argc, char** argv) {
        CliOptions opts;
        bool robots_given = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value            = [&](const char* flag) -> std::string {
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "%s requires a value\n", flag);
                    std::exit(1);
                }
                return argv[++i];
            };
            if (arg == "--headless") {
                opts.headless = true;
            }
            else if (arg == "--field") {
                opts.field = value("--field");
            }
            else if (arg == "--game") {
                opts.game = std::stoi(value("--game"));
                if (opts.game < 1 || opts.game > MAX_GAME_SIZE) {
                    std::fprintf(stderr, "--game must be between 1 and %d\n", MAX_GAME_SIZE);
                    std::exit(1);
                }
            }
            else if (arg == "--on-field-positions") {
                opts.on_field_positions = true;
            }
            else if (arg == "--config-dir") {
                opts.config_dir = value("--config-dir");
            }
            else if (arg == "--keyframe") {
                opts.keyframe = value("--keyframe");
            }
            else if (arg == "--rtf") {
                opts.rtf = std::stod(value("--rtf"));
            }
            else if (arg == "--robots") {
                opts.robots  = std::stoi(value("--robots"));
                robots_given = true;
                if (opts.robots < 1 || opts.robots > MAX_ROBOTS) {
                    std::fprintf(stderr, "--robots must be between 1 and %d\n", MAX_ROBOTS);
                    std::exit(1);
                }
            }
            else if (arg == "--web") {
                opts.web = true;
            }
            else if (arg == "--web-host") {
                opts.web_host = value("--web-host");
            }
            else if (arg == "--web-port") {
                opts.web_port = std::stoi(value("--web-port"));
                if (opts.web_port < 1 || opts.web_port > 65535) {
                    std::fprintf(stderr, "--web-port must be between 1 and 65535\n");
                    std::exit(1);
                }
            }
            else if (arg == "--referee") {
                opts.referee = true;
            }
            else if (arg == "--gamecontroller" || arg == "--gc") {
                opts.gamecontroller = true;
            }
            else if (arg == "--record") {
                opts.record = value("--record");
            }
            else if (arg == "--help" || arg == "-h") {
                std::printf(
                    "k1_mujoco_sim — MuJoCo simulator for the Booster K1 (Booster SDK DDS surface)\n"
                    "  --headless          run without the viewer window\n"
                    "  --field <name>      field to play on, from simulation.yaml's fields:\n"
                    "                        middle    RoboCup 2026 HSL M-Field, 14 x 9 m (default)\n"
                    "                        kidsize   RoboCup KidSize (pre-2026 rules), 9 x 6 m\n"
                    "                        no-field  bare flat floor, no field or ball\n"
                    "  --game <n>          a match of n robots a side, 1-11, on the middle field\n"
                    "                      (not with --field/--robots), lined up on the\n"
                    "                      touchlines; only the main robot (team 1's first) is\n"
                    "                      controlled, the rest are PD-held at the ready pose\n"
                    "  --on-field-positions  with --game, start in kickoff positions instead:\n"
                    "                      attacker, goalkeeper, two wings, then spread through the half\n"
                    "  --config-dir <dir>  config directory (default: mujoco/config)\n"
                    "  --keyframe <name>   startup keyframe (default: ready; e.g. lying_front)\n"
                    "  --rtf <factor>      real-time factor; 0 = free-run\n"
                    "  --robots <n>        total K1s on the field, 1-20 (default 1); extra robots\n"
                    "                      are uncontrolled and PD-held at the ready pose\n"
                    "  --web               serve the browser 3D viewer (works with --headless)\n"
                    "  --web-host <addr>   address the browser viewer listens on (default 127.0.0.1)\n"
                    "  --web-port <port>   port the browser viewer listens on (default 8080)\n"
                    "  --record <file>     record the match to a file, replayable in the browser viewer\n"
                    "  --referee           run the GameController inside the sim, operated from the browser\n"
                    "                      viewer's referee panel (see config/referee.yaml; implies --web)\n"
                    "  --gamecontroller    obey the RoboCup GameController: ball and robot placement, penalties\n"
                    "                      (same as supervisor.yaml enabled: true; --gc for short)\n");
                std::exit(0);
            }
            else {
                std::fprintf(stderr, "unknown argument '%s' (see --help)\n", arg.c_str());
                std::exit(1);
            }
        }
        if (opts.game > 0 && (!opts.field.empty() || robots_given)) {
            std::fprintf(stderr, "--game sets the field and robots itself; drop --field/--robots\n");
            std::exit(1);
        }
        if (opts.referee && opts.gamecontroller) {
            std::fprintf(stderr, "--referee is the GameController itself; drop --gamecontroller\n");
            std::exit(1);
        }
        opts.web = opts.web || opts.referee;
        if (opts.on_field_positions && opts.game == 0) {
            std::fprintf(stderr, "--on-field-positions needs --game\n");
            std::exit(1);
        }
        return opts;
    }

}  // namespace k1sim

#endif  // K1SIM_SHARED_CLIOPTIONS_HPP
