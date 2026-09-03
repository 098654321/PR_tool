#pragma once

#include "route_status.hh"
#include "std/string.hh"
#include <memory>
#include <optional>

namespace PR_tool::hardware {
    class Interposer;
}

namespace PR_tool::circuit {
    class BaseDie;
}

namespace PR_tool::algo {

enum class RouterKind { Maze, Sat };

struct SatRouterCliOptions {
    int verbose_level{0};
    bool enable_sat_log{false};
    std::size_t max_rss_mb{0};
    int initial_scope_pad{0};
    int initial_delay_pad{0};
    bool enable_ilp_optimize{false};
    std::optional<double> ilp_stretch_threshold_percent;
    std::optional<int> ilp_segment_bbox_pad;
    std::optional<double> ilp_time_limit_hours;
    std::String gurobi_log_dir{"gurobi"};
};

struct RouterOptions {
    RouterKind kind{RouterKind::Maze};
    int mode{0};
    bool try_all_modes{false};
    std::optional<int> compare;
    std::StringView config_path;
    SatRouterCliOptions sat {};
};

struct RouterBackend {
    virtual ~RouterBackend() = default;
    virtual auto run(
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        const RouterOptions& options
    ) -> RouteStatus = 0;
};

auto make_router(RouterKind kind) -> std::unique_ptr<RouterBackend>;

}
