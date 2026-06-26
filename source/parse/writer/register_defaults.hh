#pragma once

#include <string_view>

namespace PR_tool::parse {

    inline constexpr auto k_zero_default_hex = "00000000";
    inline constexpr auto k_ff_default_hex = "ffffffff";

    inline auto is_digit_in_range(char c, char lo, char hi) -> bool
    {
        return c >= lo && c <= hi;
    }

    inline auto is_tob_coord_digit(char c) -> bool
    {
        return is_digit_in_range(c, '0', '3');
    }

    inline auto is_ff_default_register(std::string_view name) -> bool
    {
        if (!name.starts_with("tob_")) {
            return false;
        }

        const auto rest = name.substr(4);
        const auto first_sep = rest.find('_');
        if (first_sep == std::string_view::npos || first_sep != 1) {
            return false;
        }

        const auto row = rest[0];
        const auto after_row = rest.substr(first_sep + 1);
        const auto second_sep = after_row.find('_');
        if (second_sep == std::string_view::npos || second_sep != 1) {
            return false;
        }

        const auto col = after_row[0];
        if (!is_tob_coord_digit(row) || !is_tob_coord_digit(col)) {
            return false;
        }

        const auto suffix = after_row.substr(second_sep + 1);

        if (suffix.starts_with("track2tob_")) {
            const auto index = suffix.substr(std::string_view{"track2tob_"}.size());
            return index.size() == 1 && is_tob_coord_digit(index[0]);
        }

        if (suffix.starts_with("tob2bump_bank")) {
            const auto bank_part = suffix.substr(std::string_view{"tob2bump_bank"}.size());
            return bank_part == "0_en_0" || bank_part == "0_en_1"
                || bank_part == "1_en_0" || bank_part == "1_en_1";
        }

        return false;
    }

    inline auto default_hex_for(std::string_view name) -> std::string_view
    {
        return is_ff_default_register(name) ? k_ff_default_hex : k_zero_default_hex;
    }

    inline auto should_omit_simplified_line(std::string_view hex, std::string_view name) -> bool
    {
        return hex == default_hex_for(name);
    }

} // namespace PR_tool::parse
