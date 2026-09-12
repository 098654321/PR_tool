#pragma once

#include <chrono>
#include <std/integer.hh>

namespace PR_tool {

class Elapsed {
public:
    static void start() { t0_ = clock::now(); }
    static auto milliseconds() -> std::i64 {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - t0_).count();
    }

private:
    using clock = std::chrono::steady_clock;
    static inline clock::time_point t0_{};
};

}
