#pragma once

#include <QColor>
#include <QString>

namespace PR_tool::widget {

    /// Semantic chrome colours. One accent; success is never the CTA blue.
    struct ChromeTokens {
        static constexpr const char* bg = "#F5F5F7";
        static constexpr const char* surface = "#FFFFFF";
        static constexpr const char* panel = "#FAFAFA";
        static constexpr const char* text = "#1D1D1F";
        static constexpr const char* textMuted = "#86868B";
        static constexpr const char* border = "#E5E5E5";
        static constexpr const char* borderStrong = "#C9C9C9";
        static constexpr const char* accent = "#0071E3";
        static constexpr const char* accentHover = "#0077ED";
        static constexpr const char* accentPressed = "#0060C2";
        static constexpr const char* onAccent = "#FFFFFF";
        static constexpr const char* disabledBg = "#C7D7EA";
        static constexpr const char* disabledText = "#AEAEB2";
        static constexpr const char* success = "#2E7D32";
        static constexpr const char* selectionFill = "#E8F4FD";
        static constexpr const char* radius = "8";
        static constexpr const char* radiusSm = "6";

        static auto color(const char* hex) -> QColor {
            return QColor{QLatin1String(hex)};
        }

        /// Replace @role placeholders. Longer names first so @accent does not
        /// eat @accentHover.
        static auto applyToQss(QString qss) -> QString {
            const char* const pairs[][2] = {
                {"@accentPressed", accentPressed},
                {"@selectionFill", selectionFill},
                {"@borderStrong", borderStrong},
                {"@disabledText", disabledText},
                {"@accentHover", accentHover},
                {"@disabledBg", disabledBg},
                {"@textMuted", textMuted},
                {"@onAccent", onAccent},
                {"@radiusSm", radiusSm},
                {"@success", success},
                {"@surface", surface},
                {"@accent", accent},
                {"@border", border},
                {"@radius", radius},
                {"@panel", panel},
                {"@text", text},
                {"@bg", bg},
            };
            for (const auto& pair : pairs) {
                qss.replace(QLatin1String(pair[0]), QLatin1String(pair[1]));
            }
            return qss;
        }
    };

}
