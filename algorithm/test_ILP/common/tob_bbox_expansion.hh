#pragma once

#include <algorithm>
#include <cstddef>
#include <format>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

inline constexpr std::size_t kTobBBoxMaxExpand = 4;

struct TobBBoxExpansionState {
    std::Vector<std::size_t> rho_by_record;

    static auto initial(std::size_t record_count) -> TobBBoxExpansionState {
        return TobBBoxExpansionState {std::Vector<std::size_t>(record_count, 0)};
    }

    static auto uniform(std::size_t record_count, std::size_t rho) -> TobBBoxExpansionState {
        rho = std::min(rho, kTobBBoxMaxExpand);
        return TobBBoxExpansionState {std::Vector<std::size_t>(record_count, rho)};
    }

    static auto from_vector(
        std::size_t record_count,
        const std::Vector<std::size_t>& rho_values,
        std::size_t fallback_rho = 0
    ) -> TobBBoxExpansionState {
        if (rho_values.size() != record_count) {
            return uniform(record_count, fallback_rho);
        }
        auto out = TobBBoxExpansionState {rho_values};
        for (auto& rho : out.rho_by_record) {
            rho = std::min(rho, kTobBBoxMaxExpand);
        }
        return out;
    }

    auto size() const -> std::size_t {
        return rho_by_record.size();
    }

    auto rho_for_record(std::size_t record_index) const -> std::size_t {
        if (record_index >= rho_by_record.size()) {
            return 0;
        }
        return rho_by_record[record_index];
    }

    auto max_rho() const -> std::size_t {
        if (rho_by_record.empty()) {
            return 0;
        }
        return *std::max_element(rho_by_record.begin(), rho_by_record.end());
    }

    auto expand_records(const std::Vector<std::size_t>& record_indices) -> std::Vector<std::size_t> {
        auto unique_indices = record_indices;
        std::sort(unique_indices.begin(), unique_indices.end());
        unique_indices.erase(std::unique(unique_indices.begin(), unique_indices.end()), unique_indices.end());

        auto changed = std::Vector<std::size_t> {};
        for (const auto idx : unique_indices) {
            if (idx >= rho_by_record.size()) {
                continue;
            }
            auto& rho = rho_by_record[idx];
            if (rho >= kTobBBoxMaxExpand) {
                continue;
            }
            ++rho;
            changed.push_back(idx);
        }
        return changed;
    }

    auto rho_summary() const -> std::String {
        auto out = std::String {"["};
        for (std::size_t i = 0; i < rho_by_record.size(); ++i) {
            if (i != 0) {
                out += ",";
            }
            out += std::format("{}:{}", i, rho_by_record[i]);
        }
        out += "]";
        return out;
    }
};

} // namespace PR_tool
