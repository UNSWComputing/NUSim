#pragma once

#include <cerrno>
#include <csignal>
#include <cstdio>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace k1sim::module {
    // Raw frames go directly to an encoder; no desktop capture or temporary frame files.
    class VideoRecorder {
    public:
        static constexpr int width = 1280, height = 720, fps = 15;
        explicit VideoRecorder(const std::string& path) {
            int channel[2];
            if (pipe(channel) != 0)
                throw std::runtime_error("video pipe failed");
            std::signal(SIGPIPE, SIG_IGN);
            std::vector<std::string> arguments = {"ffmpeg",     "-y",          "-hide_banner",
                                                  "-loglevel",  "error",       "-nostdin",
                                                  "-f",         "rawvideo",    "-pixel_format",
                                                  "rgb24",      "-video_size", "1280x720",
                                                  "-framerate", "15",          "-i",
                                                  "pipe:0",     "-vf",         "vflip",
                                                  "-c:v",       "libx264",     "-preset",
                                                  "veryfast",   "-threads",    "2",
                                                  "-crf",       "23",          "-pix_fmt",
                                                  "yuv420p",    "-movflags",   "+faststart",
                                                  path};
            std::vector<char*> argv;
            for (auto& argument : arguments)
                argv.push_back(argument.data());
            argv.push_back(nullptr);
            posix_spawn_file_actions_t actions;
            posix_spawn_file_actions_init(&actions);
            posix_spawn_file_actions_adddup2(&actions, channel[0], STDIN_FILENO);
            posix_spawn_file_actions_addclose(&actions, channel[0]);
            posix_spawn_file_actions_addclose(&actions, channel[1]);
            const int error = posix_spawnp(&child, "ffmpeg", &actions, nullptr, argv.data(), environ);
            posix_spawn_file_actions_destroy(&actions);
            close(channel[0]);
            if (error) {
                close(channel[1]);
                throw std::runtime_error("could not start ffmpeg");
            }
            stream = fdopen(channel[1], "wb");
            if (!stream) {
                close(channel[1]);
                finish();
                throw std::runtime_error("video stream failed");
            }
        }
        pid_t pid() const {
            return child;
        }
        ~VideoRecorder() {
            finish();
        }
        VideoRecorder(const VideoRecorder&)            = delete;
        VideoRecorder& operator=(const VideoRecorder&) = delete;
        void write(const std::vector<unsigned char>& rgb) {
            if (!stream || std::fwrite(rgb.data(), 1, rgb.size(), stream) != rgb.size())
                throw std::runtime_error("ffmpeg stopped accepting video frames");
        }
        bool finish() {
            if (stream) {
                std::fclose(stream);
                stream = nullptr;
            }
            if (child <= 0)
                return true;
            int status = 0;
            pid_t result;
            do {
                result = waitpid(child, &status, 0);
            } while (result < 0 && errno == EINTR);
            child = -1;
            return result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
        }

    private:
        std::FILE* stream = nullptr;
        pid_t child       = -1;
    };
}  // namespace k1sim::module
