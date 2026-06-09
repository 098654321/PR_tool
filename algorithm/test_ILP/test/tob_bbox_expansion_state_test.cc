#include "common/tob_bbox_expansion.hh"

#include <cassert>
#include <vector>

using namespace PR_tool;

int main() {
    auto state = TobBBoxExpansionState::initial(3);
    assert(state.size() == 3);
    assert(state.max_rho() == 0);
    assert(state.rho_for_record(0) == 0);
    assert(state.rho_for_record(2) == 0);

    const auto first = state.expand_records(std::Vector<std::size_t> {0, 2, 9});
    assert((first == std::Vector<std::size_t> {0, 2}));
    assert(state.rho_for_record(0) == 1);
    assert(state.rho_for_record(1) == 0);
    assert(state.rho_for_record(2) == 1);
    assert(state.max_rho() == 1);

    const auto duplicate = state.expand_records(std::Vector<std::size_t> {2, 2, 2});
    assert((duplicate == std::Vector<std::size_t> {2}));
    assert(state.rho_for_record(2) == 2);

    for (int i = 0; i < 8; ++i) {
        (void)state.expand_records(std::Vector<std::size_t> {0});
    }
    assert(state.rho_for_record(0) == kTobBBoxMaxExpand);
    const auto capped = state.expand_records(std::Vector<std::size_t> {0});
    assert(capped.empty());

    const auto from_vec = TobBBoxExpansionState::from_vector(2, std::Vector<std::size_t> {3, 99});
    assert(from_vec.rho_for_record(0) == 3);
    assert(from_vec.rho_for_record(1) == kTobBBoxMaxExpand);

    const auto fallback = TobBBoxExpansionState::from_vector(2, std::Vector<std::size_t> {1}, 2);
    assert(fallback.rho_for_record(0) == 2);
    assert(fallback.rho_for_record(1) == 2);
}
