#include "./log.hh"
#include <std/file.hh>

#include <mutex>
#include <string>
#include <vector>

namespace PR_tool::log {

    static auto log_file = std::OutFile{};
    static bool is_first_initialization = true;
    static std::mutex log_mutex {};
    static thread_local auto thread_prefix_stack = std::vector<std::string> {};

    static auto log(std::StringView level, std::StringView message) -> void;
    static auto log_date_time() -> void;
    static auto log_level(std::StringView level) -> void;
    static auto log_enable() -> bool;

    auto initial(const std::FilePath& log_path) -> void {
        if (log_file.is_open()) {
            log_file.close();
        }

        if (is_first_initialization) {
            log_file.open(log_path);
            is_first_initialization = false;
        } else {
            log_file.open(log_path, std::ios::app);
        }
    }

    auto debug(std::StringView message) -> void {
        if (!log_enable()) return;
        log(" DEBUG ", message);
    }

    auto info(std::StringView message) -> void {
        if (!log_enable()) return;
        log(" INFO  ", message);
    }

    auto warning(std::StringView message) -> void {
        if (!log_enable()) return;
        log("WARNING", message);
    }

    auto error(std::StringView message) -> void {
        if (!log_enable()) return;
        log(" ERROR ", message);
    }

    auto fatal(std::StringView message) -> void {
        if (!log_enable()) return;
        log(" FATAL ", message);
    }

    auto push_thread_prefix(const std::String prefix) -> void {
        thread_prefix_stack.push_back(std::string {prefix});
    }

    auto pop_thread_prefix() -> void {
        if (!thread_prefix_stack.empty()) {
            thread_prefix_stack.pop_back();
        }
    }

    auto current_thread_prefix() -> std::StringView {
        if (thread_prefix_stack.empty()) {
            return {};
        }
        return thread_prefix_stack.back();
    }

    static auto log(std::StringView level, std::StringView message) -> void
    {
        const std::lock_guard lock {log_mutex};
        log_date_time();
        log_level(level);
        log_file << message << '\n';
        log_file.flush();
    }

    static auto log_date_time() -> void
    {
        time_t current_ticks = std::time(NULL);
        struct tm* time = std::localtime(&current_ticks);

        char dateTime[32] = {0};
        std::memset(dateTime, 0, sizeof(dateTime));
        std::strftime(dateTime, sizeof(dateTime), "%Y-%m-%d %H:%M:%S", time);
        log_file << '[' << dateTime << ']';
    }

    static auto log_level(std::StringView level) -> void
    {
        log_file << ' ' << level << " > ";
    }

    static auto log_enable() -> bool {
        return log_file.is_open();
    }

}
