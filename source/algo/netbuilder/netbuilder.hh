
#pragma once

#include <hardware/track/trackcoord.hh>

#include <std/collection.hh>
#include <std/memory.hh>
#include <std/utility.hh>
#include <std/string.hh>

namespace PR_tool::hardware {
    class Track;
    class Bump;
    class Interposer;
}

namespace PR_tool::circuit {
    class BaseDie;
    class Net;
    class Connection;
    class TopDieInstance;
    class Pin;
}

namespace PR_tool::algo {

    void build_nets(circuit::BaseDie* basedie, hardware::Interposer* interposer);

    class NetBuilder {
    public:
        NetBuilder(circuit::BaseDie* basedie, hardware::Interposer* interposer);

    public:
        void build();

    private:
        auto build_no_sync_nets(
            std::Span<const std::Box<circuit::Connection>> connections,
            int group,
            int m,
            std::Vector<hardware::Bump*>& bumps_with_pose,
            std::Vector<hardware::Bump*>& bumps_with_nege
        ) -> void;
        auto build_sync_net(std::Span<const std::Box<circuit::Connection>> connections, int group, int m) -> void;
        auto build_fixed_nets(
            int m,
            const std::Vector<hardware::Bump*>& bumps_with_pose,
            const std::Vector<hardware::Bump*>& bumps_with_nege
        ) -> void;
        auto build_01_ports() -> void;
        auto register_net_at_bump_owners(
            circuit::Net* net,
            const std::Vector<hardware::Bump*>& bumps
        ) -> void;
        auto make_uid_prefix(int mode, int group, std::StringView type_token) const -> std::String;
        auto bump_uid_token(const hardware::Bump* bump) const -> std::String;
        auto track_uid_token(const hardware::Track* track) const -> std::String;
        auto sort_uid_tokens(std::Vector<std::String>& tokens) const -> void;
     
        using Node = std::Variant<hardware::Track*, hardware::Bump*>;
        auto pin_to_node(const circuit::Pin& pin) -> Node;
        

    private:
        circuit::BaseDie* _basedie {nullptr};
        hardware::Interposer* _interposer {nullptr};

        // Temp var while build!
        std::Vector<hardware::TrackCoord> _pose_tracks;
        std::Vector<hardware::TrackCoord> _nege_tracks;
        std::HashMap<hardware::Bump*, circuit::TopDieInstance*> _bump_to_topdie_inst {};
    };

}
