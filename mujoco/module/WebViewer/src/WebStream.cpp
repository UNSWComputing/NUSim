#include "module/WebViewer/src/WebStream.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace k1sim::web {

    namespace {

        constexpr const char* MAGIC          = "RCSSMJR1";
        constexpr std::size_t MAX_BACKLOG    = 400;  // pending messages before a viewer counts as stuck and is dropped
        constexpr int MAX_VISIBLE_GEOM_GROUP = 3;

#ifdef MSG_NOSIGNAL
        constexpr int SEND_FLAGS = MSG_NOSIGNAL;
#else
        constexpr int SEND_FLAGS = 0;  // macOS: SO_NOSIGPIPE is set on the socket instead
#endif

        void put_u32(std::string& out, uint32_t value) {
            for (int shift = 0; shift < 32; shift += 8) {
                out.push_back(static_cast<char>((value >> shift) & 0xFFU));
            }
        }

        // Frames a payload as <uint8 kind> <uint32 length> <payload>.
        std::string framed(uint8_t kind, const std::string& payload) {
            std::string out;
            out.reserve(payload.size() + 5);
            out.push_back(static_cast<char>(kind));
            put_u32(out, static_cast<uint32_t>(payload.size()));
            out += payload;
            return out;
        }

        std::string base64(const void* data, std::size_t size) {
            static const char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            const auto* bytes         = static_cast<const unsigned char*>(data);
            std::string out;
            out.reserve((size + 2) / 3 * 4);
            for (std::size_t i = 0; i < size; i += 3) {
                const uint32_t b0 = bytes[i];
                const uint32_t b1 = i + 1 < size ? bytes[i + 1] : 0;
                const uint32_t b2 = i + 2 < size ? bytes[i + 2] : 0;
                const uint32_t n  = (b0 << 16) | (b1 << 8) | b2;
                out.push_back(table[(n >> 18) & 63]);
                out.push_back(table[(n >> 12) & 63]);
                out.push_back(i + 1 < size ? table[(n >> 6) & 63] : '=');
                out.push_back(i + 2 < size ? table[n & 63] : '=');
            }
            return out;
        }

        // Robots of the same type share their meshes, so meshes are identified by content (FNV-1a).
        std::string content_key(const void* a, std::size_t a_size, const void* b, std::size_t b_size) {
            uint64_t hash = 1469598103934665603ULL;
            auto mix      = [&hash](const void* data, std::size_t size) {
                const auto* bytes = static_cast<const unsigned char*>(data);
                for (std::size_t i = 0; i < size; ++i) {
                    hash = (hash ^ bytes[i]) * 1099511628211ULL;
                }
            };
            mix(a, a_size);
            mix(b, b_size);
            char text[17];
            std::snprintf(text, sizeof(text), "%016llx", static_cast<unsigned long long>(hash));
            return text;
        }

        std::string json_string(const char* text) {
            std::string out = "\"";
            for (const char* c = text != nullptr ? text : ""; *c != '\0'; ++c) {
                if (*c == '"' || *c == '\\') {
                    out.push_back('\\');
                }
                if (static_cast<unsigned char>(*c) >= 0x20) {
                    out.push_back(*c);
                }
            }
            out.push_back('"');
            return out;
        }

        template <typename T>
        std::string json_array(const T* values, int count) {
            std::string out = "[";
            char text[32];
            for (int i = 0; i < count; ++i) {
                std::snprintf(text, sizeof(text), "%s%.6g", i > 0 ? "," : "", static_cast<double>(values[i]));
                out += text;
            }
            out.push_back(']');
            return out;
        }

        const char* geom_type_name(int type) {
            switch (type) {
                case mjGEOM_PLANE: return "plane";
                case mjGEOM_SPHERE: return "sphere";
                case mjGEOM_CAPSULE: return "capsule";
                case mjGEOM_ELLIPSOID: return "ellipsoid";
                case mjGEOM_CYLINDER: return "cylinder";
                case mjGEOM_BOX: return "box";
                case mjGEOM_MESH: return "mesh";
                default: return nullptr;
            }
        }

        bool send_all(int fd, const char* data, std::size_t size) {
            while (size > 0) {
                const ssize_t sent = ::send(fd, data, size, SEND_FLAGS);
                if (sent <= 0) {
                    if (sent < 0 && errno == EINTR) {
                        continue;
                    }
                    return false;  // closed, failed, or stuck for longer than the send timeout
                }
                data += sent;
                size -= static_cast<std::size_t>(sent);
            }
            return true;
        }

        bool send_all(int fd, const std::string& data) {
            return send_all(fd, data.data(), data.size());
        }

        void send_response(int fd, const char* status, const char* content_type, const std::string& body) {
            std::ostringstream head;
            head << "HTTP/1.1 " << status << "\r\nContent-Type: " << content_type << "\r\nContent-Length: " << body.size()
                 << "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
            if (send_all(fd, head.str())) {
                send_all(fd, body);
            }
        }

        const char* content_type_of(const std::string& path) {
            auto ends_with = [&path](const char* suffix) {
                const std::size_t n = std::strlen(suffix);
                return path.size() >= n && path.compare(path.size() - n, n, suffix) == 0;
            };
            if (ends_with(".html")) return "text/html; charset=utf-8";
            if (ends_with(".js")) return "text/javascript; charset=utf-8";
            if (ends_with(".css")) return "text/css; charset=utf-8";
            return "application/octet-stream";
        }

    }  // namespace

    WebStream::WebStream(Options options) : options_(std::move(options)) {}

    WebStream::~WebStream() {
        stop();
    }

    bool WebStream::start(std::string& error) {
        if (!options_.record_path.empty()) {
            record_ = std::fopen(options_.record_path.c_str(), "wb");
            if (record_ == nullptr) {
                error = "cannot write recording '" + options_.record_path + "': " + std::strerror(errno);
                return false;
            }
            std::fwrite(MAGIC, 1, std::strlen(MAGIC), record_);
        }

        running_.store(true);

        if (options_.port <= 0) {
            return true;
        }

        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listen_fd_ < 0) {
            error = std::string("socket: ") + std::strerror(errno);
            return false;
        }
        const int yes = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port   = htons(static_cast<uint16_t>(options_.port));
        if (::inet_pton(AF_INET, options_.host.c_str(), &address.sin_addr) != 1) {
            error = "invalid web host address '" + options_.host + "'";
            ::close(listen_fd_);
            listen_fd_ = -1;
            return false;
        }
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || ::listen(listen_fd_, 16) != 0) {
            error = "cannot listen on " + options_.host + ":" + std::to_string(options_.port) + ": " + std::strerror(errno);
            ::close(listen_fd_);
            listen_fd_ = -1;
            return false;
        }

        accept_thread_ = std::thread([this] { accept_loop(); });
        return true;
    }

    void WebStream::stop() {
        if (!running_.exchange(false)) {
            return;
        }

        if (accept_thread_.joinable()) {
            accept_thread_.join();  // leaves its poll() within 200 ms
        }
        if (listen_fd_ >= 0) {
            ::close(listen_fd_);
        }
        listen_fd_ = -1;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto& client : clients_) {
                std::lock_guard<std::mutex> client_lock(client->mutex);
                client->alive = false;
                ::shutdown(client->fd, SHUT_RDWR);
                client->wake.notify_all();
            }
            clients_.clear();
            if (record_ != nullptr) {
                std::fclose(record_);
                record_ = nullptr;
            }
        }

        // connection threads are detached; give them a moment to leave before members go away
        for (int i = 0; i < 200 && connections_.load() > 0; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void WebStream::set_model(const mjModel* m) {
        std::vector<Message> messages;
        std::vector<std::string> keys_by_mesh(static_cast<std::size_t>(m->nmesh));

        std::ostringstream scene;
        scene << "{\"type\":\"scene\",\"id\":" << (scene_id_ + 1) << ",\"bodies\":[";
        for (int body = 0; body < m->nbody; ++body) {
            scene << (body > 0 ? "," : "") << "{\"name\":" << json_string(mj_id2name(m, mjOBJ_BODY, body))
                  << ",\"root\":" << m->body_rootid[body] << "}";
        }
        scene << "],\"geoms\":[";

        bool first = true;
        for (int geom = 0; geom < m->ngeom; ++geom) {
            const char* type = geom_type_name(m->geom_type[geom]);
            if (type == nullptr || m->geom_group[geom] > MAX_VISIBLE_GEOM_GROUP) {
                continue;
            }

            // MuJoCo's own rule: the material colour applies unless the geom sets a colour of its own
            const int material     = m->geom_matid[geom];
            const float* geom_rgba = m->geom_rgba + 4 * geom;
            const bool default_rgba =
                geom_rgba[0] == 0.5F && geom_rgba[1] == 0.5F && geom_rgba[2] == 0.5F && geom_rgba[3] == 1.0F;
            const float* rgba = material >= 0 && default_rgba ? m->mat_rgba + 4 * material : geom_rgba;

            scene << (first ? "" : ",") << "{\"name\":" << json_string(mj_id2name(m, mjOBJ_GEOM, geom))
                  << ",\"body\":" << m->geom_bodyid[geom] << ",\"type\":\"" << type << "\",\"group\":" << m->geom_group[geom]
                  << ",\"collide\":" << (m->geom_contype[geom] != 0 || m->geom_conaffinity[geom] != 0 ? "true" : "false")
                  << ",\"size\":" << json_array(m->geom_size + 3 * geom, 3)
                  << ",\"pos\":" << json_array(m->geom_pos + 3 * geom, 3)
                  << ",\"quat\":" << json_array(m->geom_quat + 4 * geom, 4) << ",\"rgba\":" << json_array(rgba, 4)
                  << ",\"mat\":" << json_string(material >= 0 ? mj_id2name(m, mjOBJ_MATERIAL, material) : "");
            first = false;

            if (m->geom_type[geom] == mjGEOM_MESH && m->geom_dataid[geom] >= 0) {
                const int mesh   = m->geom_dataid[geom];
                std::string& key = keys_by_mesh[static_cast<std::size_t>(mesh)];
                if (key.empty()) {
                    const float* vert      = m->mesh_vert + 3 * m->mesh_vertadr[mesh];
                    const int* face        = m->mesh_face + 3 * m->mesh_faceadr[mesh];
                    const std::size_t vert_bytes = sizeof(float) * 3 * static_cast<std::size_t>(m->mesh_vertnum[mesh]);
                    const std::size_t face_bytes = sizeof(int) * 3 * static_cast<std::size_t>(m->mesh_facenum[mesh]);
                    key                    = content_key(vert, vert_bytes, face, face_bytes);

                    if (std::find(mesh_keys_.begin(), mesh_keys_.end(), key) == mesh_keys_.end()) {
                        mesh_keys_.push_back(key);
                        messages.push_back(std::make_shared<const std::string>(
                            framed(0,
                                   "{\"type\":\"mesh\",\"key\":\"" + key + "\",\"vert\":\"" + base64(vert, vert_bytes)
                                       + "\",\"face\":\"" + base64(face, face_bytes) + "\"}")));
                    }
                }
                scene << ",\"mesh\":\"" << key << "\"";
            }
            scene << "}";
        }
        scene << "]}";

        ++scene_id_;
        messages.push_back(std::make_shared<const std::string>(framed(0, scene.str())));
        for (const auto& message : messages) {
            broadcast(message, true);
        }
    }

    std::string WebStream::quote(const std::string& text) {
        return json_string(text.c_str());
    }

    void WebStream::set_game(const std::string& json) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (json == game_json_) {
                return;  // the GameController repeats its packet twice a second; only changes are worth sending
            }
            game_json_ = json;
            game_      = std::make_shared<const std::string>(framed(0, json));
        }
        broadcast(game_, false);
    }

    void WebStream::set_referee(const std::string& json) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (json == referee_json_) {
                return;
            }
            referee_json_ = json;
            referee_      = std::make_shared<const std::string>(framed(0, json));
        }
        broadcast(referee_, false);
    }

    void WebStream::set_status(const std::string& name, const std::string& json) {
        Message message;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto& entry = statuses_[name];
            if (entry.first == json) {
                return;
            }
            entry.first  = json;
            entry.second = std::make_shared<const std::string>(framed(0, json));
            message      = entry.second;
        }
        broadcast(message, false);
    }

    void WebStream::set_command_handler(std::function<bool(const std::string&, const std::string&)> handler) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_handler_ = std::move(handler);
    }

    void WebStream::publish_frame(const mjModel* m, const mjData* d) {
        if (scene_id_ == 0 || !running_.load()) {
            return;
        }

        const auto bodies = static_cast<std::size_t>(m->nbody);
        std::string payload(24 + bodies * 7 * sizeof(float), '\0');

        std::string head;
        put_u32(head, 1);
        put_u32(head, scene_id_);
        const double time = d->time;
        head.append(reinterpret_cast<const char*>(&time), sizeof(time));
        put_u32(head, static_cast<uint32_t>(bodies));
        put_u32(head, 0);
        std::memcpy(payload.data(), head.data(), 24);

        auto* poses = reinterpret_cast<char*>(payload.data()) + 24;
        for (std::size_t body = 0; body < bodies; ++body) {
            const float pose[7] = {
                static_cast<float>(d->xpos[3 * body]),      static_cast<float>(d->xpos[3 * body + 1]),
                static_cast<float>(d->xpos[3 * body + 2]),  static_cast<float>(d->xquat[4 * body]),
                static_cast<float>(d->xquat[4 * body + 1]), static_cast<float>(d->xquat[4 * body + 2]),
                static_cast<float>(d->xquat[4 * body + 3]),
            };
            std::memcpy(poses + body * sizeof(pose), pose, sizeof(pose));
        }

        broadcast(std::make_shared<const std::string>(framed(1, payload)), false);
    }

    void WebStream::broadcast(const Message& message, bool part_of_preamble) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (part_of_preamble) {
            preamble_.push_back(message);
        }
        if (record_ != nullptr) {
            std::fwrite(message->data(), 1, message->size(), record_);
            std::fflush(record_);  // keep the recording usable even if the simulator gets killed
        }

        for (const auto& client : clients_) {
            std::lock_guard<std::mutex> client_lock(client->mutex);
            if (!client->alive) {
                continue;
            }
            if (client->queue.size() > MAX_BACKLOG) {
                client->alive = false;  // too slow: never let a viewer hold back the simulation
            }
            else {
                client->queue.push_back(message);
            }
            client->wake.notify_one();
        }
        clients_.erase(std::remove_if(clients_.begin(), clients_.end(), [](const auto& c) { return !c->alive; }),
                       clients_.end());
    }

    void WebStream::accept_loop() {
        while (running_.load()) {
            // poll with a timeout rather than block in accept(): closing a socket does not wake accept() everywhere
            pollfd waiting{listen_fd_, POLLIN, 0};
            if (::poll(&waiting, 1, 200) <= 0) {
                continue;
            }
            const int fd = ::accept(listen_fd_, nullptr, nullptr);
            if (fd < 0) {
                if (errno == EINTR) {
                    continue;
                }
                break;  // listening socket closed
            }

            const int yes = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
#ifdef SO_NOSIGPIPE
            ::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
            const timeval timeout{5, 0};
            ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            connections_.fetch_add(1);
            std::thread([this, fd] {
                serve_connection(fd);
                ::close(fd);
                connections_.fetch_sub(1);
            }).detach();
        }
    }

    void WebStream::serve_connection(int fd) {
        std::string request;
        char buffer[2048];
        while (request.find("\r\n\r\n") == std::string::npos && request.size() < 16384) {
            const ssize_t received = ::recv(fd, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                return;
            }
            request.append(buffer, static_cast<std::size_t>(received));
        }

        std::istringstream line(request);
        std::string method;
        std::string path;
        line >> method >> path;
        const std::size_t question = path.find('?');
        const std::string query    = question == std::string::npos ? "" : path.substr(question + 1);
        path                       = path.substr(0, question);

        if (method == "POST") {
            std::function<bool(const std::string&, const std::string&)> handler;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                handler = command_handler_;
            }
            if (handler && handler(path, query)) {
                send_response(fd, "200 OK", "text/plain", "ok");
            }
            else {
                send_response(fd, "404 Not Found", "text/plain", "Not available.");
            }
        }
        else if (method != "GET") {
            send_response(fd, "405 Method Not Allowed", "text/plain", "Method not allowed.");
        }
        else if (path == "/stream") {
            serve_stream(fd);
        }
        else {
            serve_file(fd, path == "/" ? "/index.html" : path);
        }
    }

    void WebStream::serve_file(int fd, const std::string& path) {
        std::ifstream file;
        if (path.find("..") == std::string::npos) {
            file.open(options_.web_root + path, std::ios::binary);
        }
        if (!file.is_open()) {
            send_response(fd, "404 Not Found", "text/plain", "Not found.");
            return;
        }
        std::ostringstream body;
        body << file.rdbuf();
        send_response(fd, "200 OK", content_type_of(path), body.str());
    }

    void WebStream::serve_stream(int fd) {
        // No Content-Length: the body simply never ends, and the browser hands it to the viewer as it arrives.
        if (!send_all(fd, "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\nCache-Control: no-store\r\n"
                          "Connection: close\r\n\r\n")) {
            return;
        }

        auto client = std::make_shared<Client>();
        client->fd  = fd;
        client->queue.push_back(std::make_shared<const std::string>(MAGIC));
        {
            // registering under the lock guarantees the viewer misses nothing between the preamble and live frames
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_.load()) {
                return;
            }
            client->queue.insert(client->queue.end(), preamble_.begin(), preamble_.end());
            if (game_) {
                client->queue.push_back(game_);
            }
            if (referee_) {
                client->queue.push_back(referee_);
            }
            for (const auto& status : statuses_) {
                client->queue.push_back(status.second.second);
            }
            clients_.push_back(client);
        }

        for (;;) {
            Message message;
            {
                std::unique_lock<std::mutex> lock(client->mutex);
                client->wake.wait(lock, [&client] { return !client->alive || !client->queue.empty(); });
                if (!client->alive) {
                    break;
                }
                message = client->queue.front();
                client->queue.pop_front();
            }
            if (!send_all(fd, *message)) {
                break;
            }
        }

        std::lock_guard<std::mutex> lock(client->mutex);
        client->alive = false;  // broadcast() drops it from the list on its next pass
    }

}  // namespace k1sim::web
