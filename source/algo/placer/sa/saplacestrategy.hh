#pragma once

#include "../placestrategy.hh"
#include <circuit/topdieinst/topdieinst.hh>
#include <std/utility.hh>
#include <std/integer.hh>
#include <debug/debug.hh>

namespace PR_tool::algo {

    struct SearchBudget {
        std::size_t solve_num;
        std::size_t max_no_improvement;
    };

    class SAPlaceStrategy : public PlaceStrategy {
    public:
        SAPlaceStrategy(
            double init_temp = 100.0,
            double freeze_temp = 0.5,
            std::size_t solve_num = 50,
            double cooling_rate = 0.97,
            std::size_t max_no_improvement = 50
        ) : _init_temperature(init_temp),
            _freeze_temperature(freeze_temp),
            _base_solve_num(solve_num),
            _cooling_rate(cooling_rate),
            _base_max_no_improvement(max_no_improvement) {}

        virtual auto place(
            hardware::Interposer* interposer,
            std::Vector<circuit::TopDieInstance*>& topdies
        ) const -> void override;

        virtual auto evaluate_placement(
            hardware::Interposer* interposer,
            const std::Vector<circuit::TopDieInstance*>& topdies,
            circuit::BaseDie* basedie
        ) const -> std::i64 override;

    public:
        auto is_valid_placement(
            hardware::Interposer* interposer,
            const std::Vector<circuit::TopDieInstance*>& topdies
        ) const -> bool;

        auto save_current_placement(const std::Vector<circuit::TopDieInstance*>& topdies) const -> std::HashMap<circuit::TopDieInstance*, hardware::TOB*>;

        auto restore_placement(
            std::Vector<circuit::TopDieInstance*>& topdies,
            const std::HashMap<circuit::TopDieInstance*, hardware::TOB*>& placement
        ) const -> void;

        auto calculate_next_temperature(double current_temp, std::size_t iteration) const -> double;

        auto decide_to_swap_topdie_inst(double temperature) const -> bool;

    private:
        auto compute_search_budget(std::size_t n_chips) const -> SearchBudget;
        auto net_cost(circuit::Net* net) const -> std::i64;
        auto total_net_cost(const std::HashSet<circuit::Net*>& nets) const -> std::i64;
        auto randomly_choice_one_topdie_insts(std::Vector<circuit::TopDieInstance*>& topdies) const -> circuit::TopDieInstance*;
        auto randomly_choice_two_topdie_insts(std::Vector<circuit::TopDieInstance*>& topdies) const -> std::Tuple<circuit::TopDieInstance*, circuit::TopDieInstance*>;
        auto random_f64() const -> double;
        auto collect_nets(const std::Vector<circuit::TopDieInstance*>& topdies) const -> std::HashSet<circuit::Net*>;
        auto is_changable(circuit::TopDieInstance* inst, hardware::TOB* tob) const -> bool;
        auto check_nets(const std::Vector<circuit::TopDieInstance*>& topdies) const -> void;

    private:
        const double _init_temperature {100.0};
        const double _freeze_temperature {0.5};
        const std::size_t _base_solve_num {80};
        const double _cooling_rate {0.99};
        const std::size_t _base_max_no_improvement {50};
    };
}
