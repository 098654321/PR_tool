#include "./debug.hh"
#include "std/string.hh"

#include <cstdlib>
#include <debug/console.hh>
#include <debug/log.hh>

#include <mutex>
#include <std/file.hh>
#include <std/integer.hh>
#include <std/exception.hh>

namespace PR_tool::debug {

    static auto debug_level = DebugLevel::Info;
    static std::mutex emit_mutex {};

    auto format_prefixed_message(const std::StringView message) -> std::String {
        const auto prefix = log::current_thread_prefix();
        if (prefix.empty()) {
            return std::String {message};
        }
        return std::format("{} {}", prefix, message);
    }

    auto emit_info(const std::StringView message) -> void {
        const auto line = format_prefixed_message(message);
        const std::lock_guard lock {emit_mutex};
        console::info(line);
        log::info(line);
    }

    auto emit_warning(const std::StringView message) -> void {
        const auto line = format_prefixed_message(message);
        const std::lock_guard lock {emit_mutex};
        console::warning(line);
        log::warning(line);
    }

    auto emit_error(const std::StringView message) -> void {
        const auto line = format_prefixed_message(message);
        const std::lock_guard lock {emit_mutex};
        console::error(line);
        log::error(line);
    }

    auto emit_debug(const std::StringView message) -> void {
        const auto line = format_prefixed_message(message);
        const std::lock_guard lock {emit_mutex};
        console::debug(line);
        log::debug(line);
    }

    ScopedThreadLogPrefix::ScopedThreadLogPrefix(const std::String prefix) {
        log::push_thread_prefix(std::move(prefix));
        active_ = true;
    }

    ScopedThreadLogPrefix::~ScopedThreadLogPrefix() {
        if (active_) {
            log::pop_thread_prefix();
        }
    }

    static auto debug_level_to_number(DebugLevel level) -> std::i64 {
        switch (level) {
            case DebugLevel::Debug:   return 0;
            case DebugLevel::Info:    return 1;
            case DebugLevel::Warning: return 2;
            case DebugLevel::Error:   return 3;
            case DebugLevel::Fatal:   return 4;
        }
        return -1;
    }

    auto initial_log(const std::FilePath& log_path) -> void {
        log::initial(log_path);
    }

    auto set_debug_level(DebugLevel level) -> void {
        debug_level = level;
    }

    auto is_debug_level_enough(DebugLevel level) -> bool {
        return level >= debug_level;
    }

    auto debug(std::StringView message) -> void {
        if (!is_debug_level_enough(DebugLevel::Debug)) {
            return;
        }
        emit_debug(message);
    }

    auto info(std::StringView message) -> void {
        if (!is_debug_level_enough(DebugLevel::Info)) {
            return;
        }
        emit_info(message);
    }

    auto warning(std::StringView message) -> void {
        if (!is_debug_level_enough(DebugLevel::Warning)) {
            return;
        }
        emit_warning(message);
    }

    auto error(std::StringView message) -> void {
        if (!is_debug_level_enough(DebugLevel::Error)) {
            return;
        }
        emit_error(message);
    }

    [[noreturn]] auto fatal(std::StringView message) -> void {
        console::fatal(message);
        log::fatal(message);
        std::exit(-1);
    }

    auto check(bool condition, std::StringView message) -> void {
        if (!condition) {
            exception(message);
        } 
    }

    [[noreturn]] auto exception(std::StringView message) -> void {
        throw std::RunTimeError{std::String{message}};
    }

    [[noreturn]] auto exception_in(std::StringView where, std::StringView message) -> void {
        throw std::RunTimeError{std::format("{} >> {}", where, message)};
    }

    [[noreturn]] auto unreachable(std::StringView message) -> void {
        if (message.empty()) {
            fatal("Reach unreachable");
        } else {
            fatal_fmt("Reach unreachable in '{}'", message);
        }
    }

    [[noreturn]] auto unimplement(std::StringView message) -> void {
        if (message.empty()) {
            fatal("Reach unreachable");
        } else {
            fatal_fmt("Reach unimplement in '{}'", message);
        }
    }

}