#include "observation.hh"
#include <algorithm>
#include <memory>

namespace order_experiment {
void self_test() {
    using namespace PR_tool;
    hardware::Interposer interposer;
    auto* b0 = *interposer.get_bump(0, 0, 0);
    auto* b1 = *interposer.get_bump(1, 1, 0);
    auto* b2 = *interposer.get_bump(2, 2, 0);
    auto* t0 = *interposer.get_track(0, 0, hardware::TrackDirection::Horizontal, 0);
    auto* t1 = *interposer.get_track(0, 0, hardware::TrackDirection::Horizontal, 1);
    const std::HashSet<int> modes{0};
    auto check = [](bool condition, const char* message) {
        if (!condition) throw std::runtime_error(message);
    };
    for (int selected : policy_ids) {
        policy = selected;
        std::string name = "fixture", child_uid = "child";
        std::array<std::string, 8> ids{"sync", "tbs", "bts", "bbs", "bt", "tb", "bb", "tsbs"};
        std::vector<std::shared_ptr<Net>> nets;
        auto child = std::make_shared<circuit::BumpToBumpNet>(b0, b1, modes, name, child_uid);
        nets.push_back(std::make_shared<circuit::SyncNet>(
            std::vector<std::shared_ptr<circuit::BumpToBumpNet>>{child},
            std::vector<std::shared_ptr<circuit::BumpToTrackNet>>{},
            std::vector<std::shared_ptr<circuit::TrackToBumpNet>>{}, modes, name, ids[0]));
        nets.push_back(std::make_shared<circuit::TrackToBumpsNet>(t0, std::vector<hardware::Bump*>{b1,b2}, modes, name, ids[1]));
        nets.push_back(std::make_shared<circuit::BumpToTracksNet>(b0, std::vector<hardware::Track*>{t0,t1}, modes, name, ids[2]));
        nets.push_back(std::make_shared<circuit::BumpToBumpsNet>(b0, std::vector<hardware::Bump*>{b1,b2}, modes, name, ids[3]));
        nets.push_back(std::make_shared<circuit::BumpToTrackNet>(b0, t0, modes, name, ids[4]));
        nets.push_back(std::make_shared<circuit::TrackToBumpNet>(t0, b1, modes, name, ids[5]));
        nets.push_back(std::make_shared<circuit::BumpToBumpNet>(b0, b1, modes, name, ids[6]));
        nets.push_back(std::make_shared<circuit::TracksToBumpsNet>(
            std::vector<hardware::Track*>{t0,t1}, std::vector<hardware::Bump*>{b1,b2}, modes, name, ids[7]));
        for (std::size_t kind = 0; kind < nets.size(); ++kind) {
            check(kind_of(nets[kind].get()) == kind, "Wrong concrete Net type detection");
            check(nets[kind]->priority().value() == base_priority(kind), "Constructor priority differs from policy");
            nets[kind]->update_priority(0.5f);
            check(nets[kind]->priority().value() == base_priority(kind) + (kind ? 0.5f : 0.0f),
                  "Original port bias / SyncNet exception changed");
        }
        check(child->priority().value() == base_priority(6) + 0.5f, "Sync member priority update changed");
        std::vector<Net*> sorted;
        for (auto& net : nets) sorted.push_back(net.get());
        std::sort(sorted.begin(), sorted.end(), compare);
        if (policy == 2) check(compare(nets[3].get(), nets[1].get()) && compare(nets[6].get(), nets[5].get()),
                               "new2 does not implement corrected bump-first ordering");
        if (policy == 3) check(compare(nets[4].get(), nets[0].get()) && compare(nets[6].get(), nets[0].get()),
                               "new3 did not promote two-pin Net classes");
    }
    policy = 0;
}
} // namespace order_experiment
