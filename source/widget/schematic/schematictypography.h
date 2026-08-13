#pragma once

#include <QColor>
#include <QFont>
#include <QPalette>
#include <QWidget>

namespace PR_tool::widget::schematic {

    /// Ch.22 named pointSize table (initial values). System UI = QFont default.
    /// Header roles are fixed table sizes — no Far zoom step-down.
    struct SchematicTypography {
        static constexpr int TOPDIE_NAME_PT = 14;       // 13–14 semibold
        static constexpr int TOPDIE_TYPE_PT = 12;       // 11–12 medium
        static constexpr int PIN_NAME_PT = 9;           // 9–10 regular
        static constexpr int PROPERTY_LABEL_PT = 11;    // 11 secondary
        static constexpr int PROPERTY_VALUE_PT = 11;    // 11–12 regular
        static constexpr int STATUS_PT = 10;            // 10–11 secondary

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

        static auto propertyLabelFont() -> QFont {
            return withRole(PROPERTY_LABEL_PT, QFont::Normal);
        }

        static auto propertyValueFont() -> QFont {
            return withRole(PROPERTY_VALUE_PT, QFont::Normal);
        }

        static auto statusFont() -> QFont {
            return withRole(STATUS_PT, QFont::Normal);
        }

        static auto secondaryColor(const QWidget* widget) -> QColor {
            if (widget == nullptr) {
                return QColor(102, 102, 102);
            }
            return widget->palette().color(QPalette::PlaceholderText);
        }

        static void applyInspectorTitle(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(topDieTypeFont());
        }

        static void applyPropertyLabel(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(propertyLabelFont());
            auto pal = widget->palette();
            pal.setColor(QPalette::WindowText, secondaryColor(widget));
            widget->setPalette(pal);
        }

        static void applyPropertyValue(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(propertyValueFont());
        }

        static void applyStatus(QWidget* widget) {
            if (widget == nullptr) {
                return;
            }
            widget->setFont(statusFont());
            auto pal = widget->palette();
            pal.setColor(QPalette::WindowText, secondaryColor(widget));
            widget->setPalette(pal);
        }
    };

}
