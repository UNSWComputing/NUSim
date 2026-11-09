#ifndef K1SIM_MODULE_WEBVIEWER_WEBSTREAM_HPP
#define K1SIM_MODULE_WEBVIEWER_WEBSTREAM_HPP

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace k1sim::web {

    // Streams the simulation scene to browser viewers and/or a recording file.
    //
    // The stream is a sequence of messages: JSON text messages describing the meshes ("mesh") and the scene layout
    // ("scene"), followed by one binary message per frame holding the pose of every body:
    //   <uint32 kind=1> <uint32 scene id> <float64 sim time> <uint32 nbody> <uint32 pad> <float32[nbody * 7]>
    // with position (xyz) and orientation (wxyz) per body, all little-endian.
    //
    // On the wire and on disk the stream starts with the magic bytes "RCSSMJR1" and each message is framed as
    // <uint8 kind (0 = text, 1 = binary)> <uint32 length> <payload>. A recording is simply the stream written to a
    // file, so the viewer replays it with the same code that shows the live game (`./b replay <file>`).
    //
    // A tiny HTTP server delivers the viewer's static files, and the live stream as an endless download at /stream.
    // No dependency beyond MuJoCo and POSIX sockets; NUClear-free so it can be tested on its own.
    class WebStream {
    public:
        struct Options {
            std::string host = "127.0.0.1";
            int port         = 8080;   // <= 0: no HTTP server (recording only)
            std::string web_root;      // directory holding index.html and viewer.js
            std::string record_path;   // empty: no recording
        };

        explicit WebStream(Options options);
        ~WebStream();

        WebStream(const WebStream&)            = delete;
        WebStream& operator=(const WebStream&) = delete;

        // Opens the recording file and the listening socket. Returns false and fills `error` on failure.
        bool start(std::string& error);

        // Describes the model's meshes and scene layout. Call once the model is loaded (and again if it is replaced).
        void set_model(const mjModel* model);

        // Publishes the referee state: a JSON object {"type":"game", ...} (see module::WebViewer). Repeats are dropped.
        void set_game(const std::string& json);

        // Publishes what the built-in referee allows: a JSON object {"type":"referee", ...}. New viewers get the
        // latest one. Repeats are dropped.
        void set_referee(const std::string& json);

        // Called (on a connection thread) with the query string of every POST /referee, e.g.
        // "action=goal&side=home": a button pressed on the viewer's referee panel.
        void set_command_handler(std::function<void(const std::string&)> handler);

        // Quotes and escapes a string for use in JSON.
        static std::string quote(const std::string& text);

        // Publishes the current body poses. The caller must hold whatever lock guards `data`.
        void publish_frame(const mjModel* model, const mjData* data);

        void stop();

    private:
        using Message = std::shared_ptr<const std::string>;  // one framed message, shared between all viewers

        struct Client {
            int fd = -1;
            std::mutex mutex;
            std::condition_variable wake;
            std::deque<Message> queue;
            bool alive = true;
        };

        void accept_loop();
        void serve_connection(int fd);
        void serve_stream(int fd);
        void serve_file(int fd, const std::string& path);
        void broadcast(const Message& message, bool part_of_preamble);

        Options options_;
        int listen_fd_      = -1;
        std::FILE* record_  = nullptr;
        uint32_t scene_id_  = 0;
        std::atomic<bool> running_{false};
        std::atomic<int> connections_{0};
        std::thread accept_thread_;

        std::mutex mutex_;                              // guards clients_, preamble_ and record_
        std::vector<std::shared_ptr<Client>> clients_;
        std::vector<Message> preamble_;                 // mesh + scene messages every new viewer needs first
        std::vector<std::string> mesh_keys_;            // meshes already part of the preamble
        Message game_;                                  // latest game state, sent to new viewers after the preamble
        std::string game_json_;                         // its payload, to publish changes only
        Message referee_;                               // latest referee status, likewise
        std::string referee_json_;
        std::function<void(const std::string&)> command_handler_;
    };

}  // namespace k1sim::web

#endif  // K1SIM_MODULE_WEBVIEWER_WEBSTREAM_HPP
