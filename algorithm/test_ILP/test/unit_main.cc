#include "common/cob_unit_mask.hh"
#include "common/hw_map.hh"
#include "delay/pair_delay_precompute.hh"
#include "global_route_v17/global_guide_log.hh"
#include "global_route_v17/global_router.hh"
#include "global_route_v17/pn_source_preselection.hh"
#include "graph/unified_routing_graph.hh"
#include "post_sat_ilp/candidate_router.hh"
#include "post_sat_ilp/post_sat_ilp.hh"
#include "post_sat_rrr/post_sat_rrr.hh"
#include "sat/ideal_shortest_wirelength.hh"
#include "sat/node_occupancy.hh"
#include "sat/routing_feedback.hh"
#include "sat/routing_path_log.hh"
#include "sat/routing_round_diagnostics.hh"
#include "sat/routing_solution_validate.hh"
#include "sat/sat_constraint_kits.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/solve_unified_sat.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat/unified_sat_scope.hh"
#include "sat_allocation/cadical_solver.hh"
#include "sat_allocation/z3_optimize_solver.hh"
#include "scope/build_routing_nets.hh"
#include "scope/pair_routing_state.hh"
#include "scope/scope_bbox.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <hardware/bump/bump.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/track.hh>
#include <parse/reader/module.hh>

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>

namespace {

using namespace PR_tool;

#include "unit/common_graph_cases.inc"
#include "unit/global_route_core_cases.inc"
#include "unit/global_route_scope_cases.inc"
#include "unit/post_sat_cases.inc"
#include "unit/sat_feedback_cases.inc"
#include "unit/sat_encoding_cases.inc"
#include "unit/extract_validate_cases.inc"

} // namespace

auto main() -> int {
    try {
        test_bnet_geometry();
        test_tnet_geometry();
        test_pn_child_and_original_union();
        test_original_net_aggregation();
        test_sync_pairing_and_normalization();
        test_unified_graph_fixed_hardware_inventory();
        test_unified_graph_tob_connections();
        test_unified_graph_straight_swap_groups();
        test_unified_graph_boundary_scope();
        test_cadical_session_sat_and_counts();
        test_cadical_session_unsat();
        test_cadical_session_large_clause_sampling();
        test_cadical_session_state_guards();
        test_cadical_session_memory_limit();
        test_numeric_constraint_kits();
        test_binary_successor_truth_table();
        test_delay_precompute_simple_path();
        test_pair_state_initial_delays_bbox();
        test_delay_set_drives_d_max();
        test_cadical_assume_failed();
        test_z3_optimize_cli_option();
#ifdef USE_Z3
        test_z3_optimize_minimizes_unit_soft_clauses();
        test_z3_optimize_returns_external_assumption_core();
        test_z3_optimize_hard_unsat_has_no_alpha_core();
        test_z3_node_occupancy_matches_required_route_union();
#endif
        test_alpha_implies_sink_d();
        test_alpha_gates_sink_connectivity();
        test_alpha_skips_unreachable_delay_for_sat();
        test_alpha_empty_exact_delay_reports_core();
        test_feedback_expands_delays_on_unsat();
        test_bus_member_delay_bbox_sync();
        test_feedback_rebuilds_after_reaching_full_bbox();
        test_feedback_exhausted_state_is_unchanged();
        test_feedback_expands_only_critical_fanout_pair_delay();
        test_feedback_global_expand_skips_full_net_and_keeps_fanout_delays_independent();
        test_bus_delay_takes_max_member();
        test_v14_d_var_exact_reachability_allocation();
        test_v14_unreachable_tob_arc_has_no_a_var();
        test_v14_irrelevant_base_states_are_not_allocated();
        test_v14_d_without_predecessor_is_not_allocated();
        test_v14_reverse_reachability_prunes_dead_branch();
        test_v14_feedback_delay_rebuilds_active_mask();
        test_v14_fanout_active_mask_unions_sinks();
        test_v14_source_unit_masks();
        test_cob_unit_mask_helpers();
        test_v14_unit_mask_prunes_track_and_vline_states();
        test_v14_bnet_unit_selectors_only();
        test_v14_bnet_unit_selector_rejects_two_units();
        test_v14_pure_track_chain_sat();
        test_v14_pure_track_fork_sat();
        test_v14_pure_track_unreachable_unsat();
        test_v14_bus_equal_delay_equiv();
        test_v14_bus_forall_d_equiv();
        test_v14_bus_sync_covers_reachable_delay_domain();
        test_cli_max_rss_option();
        test_cli_time_limit_option();
        test_cli_initial_padding_options();
        test_cli_output_dir_option();
        test_v18_pn_preselection_reserves_fixed_tnet_unit_load();
        test_v18_pn_preselection_uses_unit_aware_rudy();
        test_v18_pn_preselection_scales_lambda_a_by_k_hat();
        test_v18_pn_preselection_reserves_fixed_tnet_bank_residue_load();
        test_v17_global_route_two_pin_and_fixed_unit();
        test_v17_estimated_wirelength_deduplicates_multi_terminal_channels();
        test_v22_global_route_reserves_sync_channel_unit_capacity();
        test_v19_bbox_plus_one_global_route_scope();
        test_v18_capacity_cuts_repair_overloaded_incumbent();
        test_v17_global_route_bus_and_pn_source();
        test_v17_pn_net_z_deduplicates_per_demand_channels();
        test_v17_pn_z_deduplicates_across_units();
        test_v17_pn_z_does_not_relax_per_demand_capacity();
        test_v17_channel_graph_collapses_real_hardware_topology();
        test_v17_tob_halves_share_channel_capacity();
        test_v17_rejects_tob_necessary_condition_overflow();
        test_v21_tob_peak_objective();
        test_v17_guide_scope_unit_release();
        test_v17_unit_assumption_is_traceable();
        test_v17_distance_domain_starts_at_scoped_minimum();
        test_v20_track_targets_start_at_scoped_minimum();
        test_v17_bus_distance_domain_starts_at_shared_minimum();
        test_scoped_path_error_identifies_pair();
        test_v19_tob_repair_template_and_shared_guide();
        test_v20_target_scopes_expand_guide_and_tob_patch_one_hop();
        test_v19_global_guide_feedback_thresholds_and_union();
        test_v20_global_guide_logger_handles_walk_and_residual();
        test_v22_candidate_ilp_uses_full_fixed_unit_graph();
        test_v22_two_pin_second_candidate_length_policy();
        test_v22_candidate_ilp_selects_nonconflicting_routes();
        test_v22_candidate_ilp_keeps_sync_only_congestion_fixed();
        test_v22_congestion_pool_keeps_equal_length_segment_detour();
        test_v22_multi_terminal_candidate_is_a_whole_tree();
        test_v22_multi_terminal_moves_a_whole_tree_segment();
        test_v22_ilp_optimize_chains_maze_then_full_space_ilp();
        test_v22_full_post_guide_maze_rebuilds_all_non_sync();
        test_v22_full_post_guide_maze_preserves_pn_physical_source();
        test_v20_post_sat_rrr_improves_within_final_scope();
        test_v20_post_sat_rrr_keeps_sync_as_hard_obstacle();
        test_v20_post_sat_rrr_repairs_non_sync_overflow_cascade();
        test_v17_pn_scope_retains_selected_union_only();
        test_v18_guided_cadical_smoke();
        test_initial_search_padding_scope();
        test_initial_search_padding_delay();
        test_initial_search_padding_rejects_delay_overflow();
        test_initial_search_padding_fanout_keeps_pair_delays_independent();
        test_initial_search_padding_bus_uses_shared_delay_without_bbox_merge();
        test_initial_search_padding_applies_scope_before_delay();
        test_sequential_at_most_one_truth_table_and_scaling();
        test_numeric_fixed_path_and_extraction();
        test_v14_rejects_multi_candidate_demand();
        test_initial_search_padding_pnnet_keeps_sink_delays_independent();
        test_pnnet_virtual_delay_shift();
        test_pnnet_forbid_track_delay_ne_1();
        test_pnnet_virtual_arc_sat_extract();
        test_logical_source_exclusivity();
        test_logical_source_owns_its_source_node();
        test_sat_encoding_stats_reconcile();
        test_v16_occupancy_stats_are_not_auxiliary();
        test_source_distance_constants();
        test_mode_group_zero_conflict();
        test_all_mode_groups_and_group_zero_extraction();
        test_y_aggregation_and_partial_matching();
        test_all_four_y_partial_matching_sides();
        test_y_bidirectional_equivalence();
        test_forward_and_reverse_switch_use_extract_same_y();
        test_extraction_selects_one_valid_predecessor();
        test_extraction_handles_shared_source_fanout();
        test_extraction_error_branches();
        test_format_path_node_track_bump_hline_vline();
        test_infer_net_display_kind();
        test_format_bbox_corners();
        test_path_wirelength_counts_bump_and_track_only();
        test_format_path_hops_and_graph_node_ref();
        test_log_routing_paths_two_pin();
        test_validate_accepts_valid_two_pin();
        test_validate_uses_encoded_guide_scope_instead_of_legacy_bbox();
        test_validate_detects_node_outside_encoded_scope();
        test_validate_detects_missing_arc();
        test_validate_detects_endpoint_mismatch();
        test_unique_failed_net_ids_deduplicates();
        test_ideal_two_pin_wirelength_matches_shortest_path();
        test_ideal_sync_bus_wirelength_scales_by_members();
        std::cout << "test_ILP_unit: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "test_ILP_unit: " << error.what() << '\n';
        return 1;
    }
}
