#include "./layoutscene.h"
#include "./item/netitem.h"
#include "./item/pinitem.h"
#include "./item/tobitem.h"
#include "./item/topdieinstitem.h"
#include "circuit/connection/pin.hh"
#include "circuit/connection/connection.hh"
#include "circuit/net/net.hh"
#include "hardware/track/trackcoord.hh"
#include "qglobal.h"
#include "qpoint.h"
#include "widget/layout/item/exportitem.h"
#include "widget/layout/item/sourceportitem.h"
#include <cassert>
#include <circuit/basedie.hh>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <hardware/interposer.hh>
#include <optional>
#include <widget/frame/itemtypecheck.h>

#include <debug/debug.hh>
#include <QDebug>
#include <QMessageBox>
#include <QGraphicsView>
#include <QSet>

namespace PR_tool::widget {

    static_assert(AllUnique<
        (int)layout::NetItem::Type,
        (int)layout::PinItem::Type,
        (int)layout::TopDieInstanceItem::Type,
        (int)layout::ExternalPortItem::Type,
        (int)layout::SourcePortItem::Type
    >::value);

    using namespace layout;

    const QPointF LayoutScene::INTERPOSER_LEFT_DOWN_POSITION = 
        LayoutScene::tobPosition(hardware::TOBCoord{0, 0}) - 
        QPointF{TOBItem::WIDTH/2 + INTERPOSER_SIDE_GAP, -TOBItem::HEIGHT/2 - INTERPOSER_SIDE_GAP};

    const QPointF LayoutScene::INTERPOSER_RIGHT_UP_POSITION = 
        LayoutScene::tobPosition(hardware::TOBCoord{hardware::Interposer::TOB_ARRAY_WIDTH-1, hardware::Interposer::TOB_ARRAY_HEIGHT-1}) + 
        QPointF{TOBItem::WIDTH/2 + INTERPOSER_SIDE_GAP, -TOBItem::HEIGHT/2 - INTERPOSER_SIDE_GAP};

    const QPointF LayoutScene::EXPORT_LEFT_DOWN_POSITION = 
        LayoutScene::tobPosition(hardware::TOBCoord{0, 0}) - 
        QPointF{TOBItem::WIDTH/2 + EXPORT_SIDE_GAP, -TOBItem::HEIGHT/2 - EXPORT_SIDE_GAP};

    const QPointF LayoutScene::EXPORT_RIGHT_UP_POSITION = 
        LayoutScene::tobPosition(hardware::TOBCoord{hardware::Interposer::TOB_ARRAY_WIDTH-1, hardware::Interposer::TOB_ARRAY_HEIGHT-1}) + 
        QPointF{TOBItem::WIDTH/2 + EXPORT_SIDE_GAP, -TOBItem::HEIGHT/2 - EXPORT_SIDE_GAP};


    LayoutScene::LayoutScene(hardware::Interposer* interposer, circuit::BaseDie* basedie, QObject* parent) :
        QGraphicsScene{parent},
        _basedie{basedie},
        _interposer{interposer}
    {
        this->addSceneItems();
    }

    void LayoutScene::reloadItems() {
        this->_highlightedTOB = nullptr;
        this->_highlightedTopDie = nullptr;
        this->_topdieinstMap.clear();
        this->_externalPortsMap.clear();
        this->_tobsMaps.clear();
        this->_nets.clear();
        this->_vddPorts.clear();
        this->_gndPorts.clear();
        this->_netsWithSourcePorts.clear();
        this->clear();

        this->addSceneItems();
    }

    void LayoutScene::clearTopDieHighlight() {
        if (this->_highlightedTOB != nullptr) {
            this->_highlightedTOB->highlight(false);
            this->_highlightedTOB = nullptr;
        }
        if (this->_highlightedTopDie != nullptr) {
            this->_highlightedTopDie->highlight(false);
            this->_highlightedTopDie = nullptr;
        }
    }

    void LayoutScene::focusTopDieInstance(const QString& name) {
        this->clearTopDieHighlight();
        if (name.isEmpty()) {
            return;
        }

        for (auto it = this->_topdieinstMap.cbegin(); it != this->_topdieinstMap.cend(); ++it) {
            auto* inst = it.key();
            auto* instItem = it.value();
            if (inst == nullptr || instItem == nullptr) {
                continue;
            }
            if (QString::fromStdString(inst->name().data()) != name) {
                continue;
            }

            auto* tobItem = this->_tobsMaps.value(inst->tob(), nullptr);
            if (tobItem != nullptr) {
                tobItem->highlight(true);
                this->_highlightedTOB = tobItem;
            }
            instItem->highlight(true);
            this->_highlightedTopDie = instItem;
            for (auto* view : this->views()) {
                view->centerOn(instItem);
            }
            return;
        }
    }

    void LayoutScene::addSceneItems() {
        this->addTOBItems();
        this->addTopDieInstanceItems();
        this->addExternalPortItems();
        this->addNetItems();

        this->choiseSourcePort();
    }

    void LayoutScene::addTOBItems() {
        for (auto& [coord, tob] : this->_interposer->tobs()) {
            auto t = this->addTOB(tob.get());
            t->setPos(LayoutScene::tobPosition(coord));
        }

        auto r = new QGraphicsRectItem {QRectF{
            INTERPOSER_LEFT_DOWN_POSITION, 
            INTERPOSER_RIGHT_UP_POSITION 
        }};
        
        r->setBrush(Qt::gray);
        r->setZValue(layout::TOBItem::Z_VALUE - 1);
        this->addItem(r);
    }

    void LayoutScene::addTopDieInstanceItems() {
        // Call after addTOBItems!!
        for (auto& [name, topdieInst] : this->_basedie->topdie_insts()) {
            auto tobItem = this->_tobsMaps.value(topdieInst->tob());
            this->addTopDieInstance(topdieInst.get(), tobItem);
        }
        this->syncDefaultPlacement();
    }

    void LayoutScene::syncDefaultPlacement() {
        QSet<circuit::TopDieInstance*> live;
        for (auto it = this->_topdieinstMap.cbegin(); it != this->_topdieinstMap.cend(); ++it) {
            auto* inst = it.key();
            if (inst == nullptr) {
                continue;
            }
            live.insert(inst);
            if (!this->_defaultPlacement.contains(inst)) {
                this->_defaultPlacement.insert(inst, inst->tob());
            }
        }
        for (auto it = this->_defaultPlacement.begin(); it != this->_defaultPlacement.end(); ) {
            if (!live.contains(it.key())) {
                it = this->_defaultPlacement.erase(it);
            } else {
                ++it;
            }
        }
    }

    void LayoutScene::restoreDefaultPlacement() {
        bool changed = false;
        const auto insts = this->_defaultPlacement.keys();
        for (auto* inst : insts) {
            auto* target = this->_defaultPlacement.value(inst, nullptr);
            if (inst == nullptr || target == nullptr || inst->tob() == target) {
                continue;
            }
            changed = true;
            if (target->is_idle()) {
                inst->move_to_tob(target);
            } else {
                auto* occupant = target->placed_instance();
                if (occupant != nullptr && occupant != inst) {
                    inst->swap_tob_with(occupant);
                }
            }
        }
        if (!changed) {
            return;
        }
        this->reloadItems();
        emit this->layoutChanged();
    }

    void LayoutScene::addExternalPortItems() {
        const auto width = EXPORT_RIGHT_UP_POSITION.x() - EXPORT_LEFT_DOWN_POSITION.x();
        const auto height = EXPORT_LEFT_DOWN_POSITION.y() - EXPORT_RIGHT_UP_POSITION.y();
        const auto width_interval = width / (hardware::Interposer::COB_ARRAY_WIDTH * (int)hardware::COB::INDEX_SIZE);
        const auto height_interval = height / (hardware::Interposer::COB_ARRAY_WIDTH * (int)hardware::COB::INDEX_SIZE);

        auto get_port_position = [width_interval, height_interval] (const hardware::TrackCoord& coord) -> QPointF {
            switch (coord.dir) {
                case hardware::TrackDirection::Horizontal: {
                    auto index = coord.row * hardware::COB::INDEX_SIZE + coord.index;
                    auto y = EXPORT_LEFT_DOWN_POSITION.y() - height_interval * index;
                    if (coord.col == 0) {
                        return QPointF { EXPORT_LEFT_DOWN_POSITION.x(), y };
                    } 
                    else {
                        return QPointF { EXPORT_RIGHT_UP_POSITION.x(), y };
                    }
                }
                case hardware::TrackDirection::Vertical: {
                    auto index = coord.col * hardware::COB::INDEX_SIZE + coord.index;
                    auto x = EXPORT_LEFT_DOWN_POSITION.x() + index * width_interval;
                    if (coord.row == 0) {
                        return QPointF { x, EXPORT_LEFT_DOWN_POSITION.y() };
                    } 
                    else {
                        return QPointF { x, EXPORT_RIGHT_UP_POSITION.y()};
                    }
                }
            }
            debug::unreachable();
        };

        for (auto& [name, eport] : this->_basedie->external_ports()) {
            auto item = this->addExternalPort(eport.get());
            const auto coord = eport->coord();
            assert(hardware::Interposer::is_external_port_coord(coord));
            item->setPos(get_port_position(coord));
        }

        for (auto& coord : this->_basedie->pose_ports()) {
            auto port = this->addVDDSourcePort();
            port->setPos(get_port_position(coord));
        }

        for (auto& coord : this->_basedie->nege_ports()) {
            auto port = this->addGNDSourcePort();
            port->setPos(get_port_position(coord));
        }
    }

    void LayoutScene::addNetItems() {
        for (auto& [mode, inner_connection]: this->_basedie->connections()) {
            for (auto& [_, connections] : inner_connection) {
                for (const auto& connection : connections) {
                    auto beginPin = this->circuitPinToPinItem(connection->input_pin());
                    auto endPin = this->circuitPinToPinItem(connection->output_pin());
                    
                    assert(beginPin != nullptr);
                    assert(endPin != nullptr);
    
                    this->addNet(beginPin, endPin);
                }
            }
        }
        
    }

    auto LayoutScene::addNet(layout::PinItem* beginPin, layout::PinItem* endPin) -> layout::NetItem* {
        // Illegal: source-to-source connection (algo unchanged; warn and skip)
        if (beginPin->isSourcePortPin() && endPin->isSourcePortPin()) {
            QWidget* parent = nullptr;
            const auto sceneViews = this->views();
            if (!sceneViews.isEmpty()) {
                parent = sceneViews.first();
            }
            QMessageBox::warning(
                parent,
                QStringLiteral("Invalid Net"),
                QStringLiteral("Cannot connect a source port to another source port (VDD/GND).")
            );
            return nullptr;
        }

        auto n = new layout::NetItem {beginPin, endPin};
        if (beginPin->isSourcePortPin() || endPin->isSourcePortPin()) {
            this->_netsWithSourcePorts.push_back(n);
        }
        this->_nets.push_back(n);
        this->addItem(n);
        return n;
    }

    auto LayoutScene::addTOB(hardware::TOB* tob) -> layout::TOBItem* {
        auto t = new layout::TOBItem {tob};
        this->_tobsMaps.insert(tob, t);
        this->addItem(t);
        return t;
    }

    auto LayoutScene::addTopDieInstance(circuit::TopDieInstance* topdieInst, layout::TOBItem* tob) -> layout::TopDieInstanceItem* {
        auto ti = new layout::TopDieInstanceItem {topdieInst, tob};
        this->_topdieinstMap.insert(topdieInst, ti);
        this->addItem(ti);
        return ti;
    }

    auto LayoutScene::addExternalPort(circuit::ExternalPort* eport) -> layout::ExternalPortItem* {
        auto eportItem = new layout::ExternalPortItem {eport};
        this->_externalPortsMap.insert(eport, eportItem);
        this->addItem(eportItem);
        return eportItem;
    }

    auto LayoutScene::addVDDSourcePort() -> layout::SourcePortItem* {
        auto portItem = new layout::SourcePortItem {layout::SourcePortType::VDD};
        this->_vddPorts.push_back(portItem);
        this->addItem(portItem);
        return portItem;
    }

    auto LayoutScene::addGNDSourcePort() -> layout::SourcePortItem* {
        auto portItem = new layout::SourcePortItem {layout::SourcePortType::GND};
        this->_gndPorts.push_back(portItem);
        this->addItem(portItem);
        return portItem;
    }

    namespace {
        auto hpwlOfCoords(const std::Vector<hardware::Coord>& coords) -> qint64 {
            if (coords.empty()) {
                return 0;
            }
            auto min_row = coords[0].row;
            auto max_row = coords[0].row;
            auto min_col = coords[0].col;
            auto max_col = coords[0].col;
            for (std::size_t i = 1; i < coords.size(); ++i) {
                min_row = std::min(min_row, coords[i].row);
                max_row = std::max(max_row, coords[i].row);
                min_col = std::min(min_col, coords[i].col);
                max_col = std::max(max_col, coords[i].col);
            }
            return (max_row - min_row) + (max_col - min_col);
        }

        auto placementCoordOfPin(const circuit::Pin& pin) -> std::optional<hardware::Coord> {
            return std::match(pin.connected_point(),
                [](const circuit::ConnectVDD&) -> std::optional<hardware::Coord> {
                    return std::nullopt;
                },
                [](const circuit::ConnectGND&) -> std::optional<hardware::Coord> {
                    return std::nullopt;
                },
                [](const circuit::ConnectExPort& eport) -> std::optional<hardware::Coord> {
                    const auto& c = eport.port->coord();
                    return hardware::Coord{c.row, c.col};
                },
                [](const circuit::ConnectBump& bump) -> std::optional<hardware::Coord> {
                    if (bump.inst == nullptr || bump.inst->tob() == nullptr) {
                        return std::nullopt;
                    }
                    return bump.inst->tob()->coord();
                }
            );
        }
    }

    auto LayoutScene::estimatedHpwlFromNets() -> qint64 {
        qint64 total = 0;
        bool any = false;
        for (const auto& [mode, nets] : this->_basedie->nets()) {
            for (const auto& net : nets) {
                any = true;
                total += hpwlOfCoords(net->coords());
            }
        }
        return any ? total : -1;
    }

    auto LayoutScene::estimatedHpwlFromConnections() -> qint64 {
        qint64 total = 0;
        for (const auto& [mode, inner] : this->_basedie->connections()) {
            for (const auto& [sync, connections] : inner) {
                for (const auto& connection : connections) {
                    auto a = placementCoordOfPin(connection->input_pin());
                    auto b = placementCoordOfPin(connection->output_pin());
                    if (!a || !b) {
                        continue;
                    }
                    total += std::llabs(a->row - b->row) + std::llabs(a->col - b->col);
                }
            }
        }
        return total;
    }

    auto LayoutScene::estimatedTotalWireLength() -> qint64 {
        // Prefer SA-style HPWL over built circuit nets; fall back to connections.
        const auto fromNets = this->estimatedHpwlFromNets();
        if (fromNets >= 0) {
            return fromNets;
        }
        return this->estimatedHpwlFromConnections();
    }

    void LayoutScene::choiseSourcePort() {
        for (auto net : this->_netsWithSourcePorts) {
            if (net->beginPin()->isSourcePortPin()) {
                auto endPinPos = net->endPin()->scenePos();
                
                auto originPort = net->beginPin()->parentSourcePort();
                auto& ports = originPort->isVDD() ? this->_vddPorts : this->_gndPorts;

                auto newBeginPort = 
                    LayoutScene::getMinDistancePort(ports, endPinPos);
                
                net->moveToBeginPin(newBeginPort->pin());
            }
            else if (net->endPin()->isSourcePortPin()) {
                auto beginPinPos = net->beginPin()->scenePos();
                
                auto originPort = net->endPin()->parentSourcePort();
                auto& ports = originPort->isVDD() ? this->_vddPorts : this->_gndPorts;

                auto newEndPort = 
                    LayoutScene::getMinDistancePort(ports, beginPinPos);
                
                net->moveToEndPin(newEndPort->pin());
            } 
            else {
                debug::unreachable();
            }
        }
    }

    auto LayoutScene::circuitPinToPinItem(const circuit::Pin& pin) -> layout::PinItem* {
        return std::match(pin.connected_point(),
            [this](const circuit::ConnectVDD& vdd) -> PinItem* {
                return this->_vddPorts.front()->pin();
            },
            [this](const circuit::ConnectGND& vdd) -> PinItem* {
                return this->_gndPorts.front()->pin();
            },
            [this](const circuit::ConnectExPort& eport) -> PinItem* {
                auto eportItem = this->_externalPortsMap.value(eport.port);
                return eportItem->pin();
            },
            [this](const circuit::ConnectBump& bump) -> PinItem* {
                auto inst = this->_topdieinstMap.value(bump.inst);
                auto pin = inst->pins()[bump.inst->topdie()->pins_map().at(bump.name)];
                return pin;
            }
        );
    }

    auto LayoutScene::getMinDistancePort(
        const QVector<layout::SourcePortItem*> ports, 
        const QPointF& targetPos) -> layout::SourcePortItem* 
    {
        auto minDistance = std::numeric_limits<qreal>::max();
        layout::SourcePortItem* minPort = nullptr;

        for (auto sport : ports) {
            auto distance = LayoutScene::pointDistance(sport->pin()->scenePos(), targetPos);
            if (distance < minDistance) {
                minDistance = distance;
                minPort = sport;
            }
        }

        assert(minPort != nullptr);
        return minPort;
    }

    auto LayoutScene::pinDistance(layout::PinItem* pin1, layout::PinItem* pin2) -> qreal {
        assert(pin1 != nullptr && pin2 != nullptr);
        return LayoutScene::pointDistance(pin1->scenePos(), pin2->scenePos());
    }

    auto LayoutScene::pointDistance(const QPointF& p1, const QPointF& p2) -> qreal {
        return (p1 - p2).manhattanLength();
    }

    auto LayoutScene::tobPosition(const hardware::TOBCoord& coord) -> QPointF {
        return QPointF{coord.col * (TOBItem::WIDTH + TOB_INTERVAL), -coord.row * (TOBItem::HEIGHT + TOB_INTERVAL)};
    }

}