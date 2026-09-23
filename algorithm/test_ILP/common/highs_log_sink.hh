#pragma once

#include <Highs.h>

#include <cstdio>
#include <debug/debug.hh>
#include <string>
#include <string_view>

namespace PR_tool {

// Writes HiGHS solver logs to a file and optionally echoes them to stdout.
class HighsLogSink {
public:
    HighsLogSink(const std::string_view path, const bool echo_console, const bool append)
        : echo_console_(echo_console) {
        if (path.empty()) {
            return;
        }
        const auto path_string = std::string {path};
        file_ = std::fopen(path_string.c_str(), append ? "a" : "w");
        if (file_ == nullptr) {
            debug::warning_fmt("failed to open HiGHS log file: {}", path_string);
        }
    }

    ~HighsLogSink() {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    HighsLogSink(const HighsLogSink&) = delete;
    auto operator=(const HighsLogSink&) -> HighsLogSink& = delete;

    auto attach(Highs& highs) -> void {
        const bool enable = file_ != nullptr || echo_console_;
        highs.setOptionValue("output_flag", enable);
        // HiGHS drops log lines when both log_stream and log_to_console are off,
        // even with an active logging callback.
        highs.setOptionValue("log_to_console", enable);
        if (!enable) {
            return;
        }
        highs.setCallback(
            [this](
                const int type,
                const std::string& message,
                const HighsCallbackOutput*,
                HighsCallbackInput*,
                void*
            ) {
                if (type != kCallbackLogging) {
                    return;
                }
                if (file_ != nullptr) {
                    std::fputs(message.c_str(), file_);
                    std::fflush(file_);
                }
                if (echo_console_) {
                    std::fputs(message.c_str(), stdout);
                    std::fflush(stdout);
                }
            });
        highs.startCallback(kCallbackLogging);
    }

private:
    std::FILE* file_{nullptr};
    bool echo_console_{false};
};

} // namespace PR_tool
