#pragma once

#include <algorithm>
#include <cstddef>
#include <format>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct TobTierState {
    std::Vector<std::size_t> tier_by_record;

    static auto initial(std::size_t record_count) -> TobTierState {
        return TobTierState {std::Vector<std::size_t>(record_count, 0)};
    }

    static auto uniform(std::size_t record_count, std::size_t tier) -> TobTierState {
        return TobTierState {std::Vector<std::size_t>(record_count, tier)};
    }

    static auto from_vector(
        std::size_t record_count,
        const std::Vector<std::size_t>& tier_values,
        std::size_t fallback_tier = 0
    ) -> TobTierState {
        if (tier_values.size() != record_count) {
            return uniform(record_count, fallback_tier);
        }
        return TobTierState {tier_values};
    }

    auto size() const -> std::size_t {
        return tier_by_record.size();
    }

    auto tier_for_record(std::size_t record_index) const -> std::size_t {
        if (record_index >= tier_by_record.size()) {
            return 0;
        }
        return tier_by_record[record_index];
    }

    auto max_tier() const -> std::size_t {
        if (tier_by_record.empty()) {
            return 0;
        }
        return *std::max_element(tier_by_record.begin(), tier_by_record.end());
    }

    auto tier_summary() const -> std::String {
        auto out = std::String {"["};
        for (std::size_t i = 0; i < tier_by_record.size(); ++i) {
            if (i != 0) {
                out += ",";
            }
            out += std::format("{}:{}", i, tier_by_record[i]);
        }
        out += "]";
        return out;
    }
};

} // namespace PR_tool
