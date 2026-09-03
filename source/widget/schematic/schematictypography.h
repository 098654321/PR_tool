#pragma once

#include "../chrometokens.h"

#include <QColor>
#include <QFont>
#include <QPalette>
#include <QWidget>

namespace PR_tool::widget::schematic {

    /// Ch.22 named pointSize table (canvas) plus P1-2 chrome roles.
    /// Do not reuse canvas pin / topdie fonts on chrome widgets.
    struct SchematicTypography {
        static constexpr int TOPDIE_NAME_PT = 64;       // centered instance name (canvas)
        static constexpr int TOPDIE_TYPE_PT = 36;       // centered type subtitle (canvas)
        static constexpr int PIN_NAME_PT = 9;           // 9–10 regular (canvas)
        static constexpr int PANEL_SECTION_PT = 11;     // 11–12 muted, open tracking
        static constexpr int TREE_PT = 12;              // tree nodes
        static constexpr int PROPERTY_LABEL_PT = 11;    // 10–11 muted
        static constexpr int PROPERTY_VALUE_PT = 12;    // 12 regular
        static constexpr int STATUS_PT = 10;            // 10–11 muted
        static constexpr int STATUS_EMPHASIS_PT = 11;   // Stage / Ready
        static constexpr int PALETTE_PT = 10;           // smaller than tree 12pt
        static constexpr qreal PANEL_SECTION_TRACKING = 112.0;

        static auto baseFont() -> QFont {
            return QFont{};
        }

        static auto withRole(int pointSize, QFont::Weight weight) -> QFont {
            auto font = baseFont();
            font.setPointSize(pointSize);
            font.setWeight(weight);
            return font;
        }

        static auto topDieNameFont() -> QFont {
            return withRole(TOPDIE_NAME_PT, QFont::DemiBold);
        }

        static auto topDieTypeFont() -> QFont {
            return withRole(TOPDIE_TYPE_PT, QFont::Medium);
        }

        static auto pinNameFont() -> QFont {
            return withRole(PIN_NAME_PT, QFont::Normal);
        }

        static auto panelSectionFont() -> QFont {
            auto font = withRole(PANEL_SECTION_PT, QFont::Normal);
            font.setLetterSpacing(QFont::PercentageSpacing, PANEL_SECTION_TRACKING);
            return font;
        }

        static auto treeFont() -> QFont {
            return withRole(TREE_PT, QFont::Normal);
        }

        static auto propertyLabelFont() -> QFont {
            return withRole(PROPERTY_LABEL_PT, QFont::Normal);
        }

        static auto propertyValueFont() -> QFont {
            return withRole(PROPERTY_VALUE_PT, QFont::Normal);
        }

        static auto statusFont() -> QFont {
            return withRole(STATUS_PT, QFont::Normal);
        }

        static auto statusEmphasisFont() -> QFont {
            return withRole(STATUS_EMPHASIS_PT, QFont::Medium);
        }

        static auto paletteFont() -> QFont {
            return withRole(PALETTE_PT, QFont::Medium);
        }

        static void applyForeground(QWidget* widget, const char* hex) {
            if (widget == nullptr) {
                return;
            }
            const auto color = ChromeTokens::color(hex);
            auto pal = widget->palette();
            pal.setColor(QPalette::WindowText, color);
            pal.setColor(QPalette::Text, color);
            pal.setColor(QPalette::ButtonText, color);
            widget->setPalette(pal);
        }

        static void applyPanelSectionTitle(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(panelSectionFont());
            applyForeground(widget, ChromeTokens::textMuted);
        }

        static void applyInspectorTitle(QWidget* widget) {
            applyPanelSectionTitle(widget);
        }

        static void applyTree(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(treeFont());
            applyForeground(widget, ChromeTokens::text);
        }

        static void applyPropertyLabel(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(propertyLabelFont());
            applyForeground(widget, ChromeTokens::textMuted);
        }

        static void applyPropertyValue(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(propertyValueFont());
            applyForeground(widget, ChromeTokens::text);
        }

        static void applyStatus(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(statusFont());
            applyForeground(widget, ChromeTokens::textMuted);
        }

        static void applyStatusEmphasis(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(statusEmphasisFont());
            applyForeground(widget, ChromeTokens::text);
        }

        static void applyStatusHint(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(statusFont());
            applyForeground(widget, ChromeTokens::disabledText);
        }

        static void applyPaletteButton(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(paletteFont());
        }
    };

}
