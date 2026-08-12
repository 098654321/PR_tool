#include "./schematicscene.h"

#include "./item/netitem.h"
#include "./item/netpointitem.h"
#include "./item/pinitem.h"
#include "./item/exportitem.h"
#include "./item/topdieinstitem.h"
#include "./item/portgroupitem.h"
#include "./item/griditem.h"
#include "./item/sourceportitem.h"
#include "./item/powerrailitem.h"

#include <circuit/connection/pin.hh>
#include <circuit/connection/connection.hh>
#include <circuit/topdieinst/topdieinst.hh>
#include <circuit/basedie.hh>
#include <hardware/interposer.hh>
#include <widget/frame/itemtypecheck.h>
#include <widget/frame/graphicsview.h>

#include <debug/debug.hh>
#include <QMessageBox>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QTimer>
#include <QHash>
#include <QPair>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace PR_tool::widget {

    static_assert(AllUnique<
        (int)schematic::NetItem::Type,
        (int)schematic::PinItem::Type,
        (int)schematic::TopDieInstanceItem::Type,
        (int)schematic::NetPointItem::Type,
        (int)schematic::ExternalPortItem::Type,
        (int)schematic::SourcePortItem::Type,
        (int)schematic::PortGroupItem::Type,
        (int)schematic::ExportPortGroupHost::Type,
        (int)schematic::PowerRailItem::Type
    >::value);

    SchematicScene::SchematicScene(circuit::BaseDie* basedie, hardware::Interposer* interposer) :
        _basedie{basedie},
        _interposer{interposer},
        QGraphicsScene{}
    {
        this->addSceneItems();
        this->refreshConnectionFocus();
    }

    void SchematicScene::reloadItems() {
        // Clear
        this->_topdieinstMap.clear();
        this->_exportMap.clear();
        this->_nets.clear();
        this->_vddPorts.clear();
        this->_gndPorts.clear();
        this->_vddRail = nullptr;
        this->_gndRail = nullptr;

        this->_floatingNet = nullptr;
        this->_floatingTopdDieInst = nullptr;
        this->_floatingExPort = nullptr;
        this->_exportGroupHost = nullptr;
        this->_pendingTopDieGroupSync.clear();
        this->_pendingExportGroupSync = false;
        this->_portGroupFlushScheduled = false;

        this->_hoverTopDie = nullptr;
        this->_selectedTopDie = nullptr;
        this->_hoverFocusNets.clear();
        this->_selectedFocusNets.clear();

        this->clear();

        this->addSceneItems();
        this->refreshConnectionFocus();
    }

    void SchematicScene::requestPortGroupSync(schematic::TopDieInstanceItem* item) {
        if (!item) {
            return;
        }
        this->_pendingTopDieGroupSync.insert(item);
        if (this->_portGroupFlushScheduled) {
            return;
        }
        this->_portGroupFlushScheduled = true;
        QTimer::singleShot(0, this, &SchematicScene::flushPortGroupSync);
    }

    void SchematicScene::requestExportPortGroupSync() {
        this->_pendingExportGroupSync = true;
        if (this->_portGroupFlushScheduled) {
            return;
        }
        this->_portGroupFlushScheduled = true;
        QTimer::singleShot(0, this, &SchematicScene::flushPortGroupSync);
    }

    void SchematicScene::flushPortGroupSync() {
        this->_portGroupFlushScheduled = false;

        const auto pendingTop = this->_pendingTopDieGroupSync;
        this->_pendingTopDieGroupSync.clear();
        for (auto* item : pendingTop) {
            if (item && this->_topdieinstMap.values().contains(item)) {
                item->syncPortGroups();
                item->update();
                for (auto* pin : item->pins()) {
                    pin->update();
                }
            }
        }

        if (this->_pendingExportGroupSync) {
            this->_pendingExportGroupSync = false;
            if (this->_exportGroupHost) {
                this->_exportGroupHost->syncGroups();
                this->_exportGroupHost->update();
            }
        }

        this->markBundleNets();
    }

    void SchematicScene::addSceneItems() {
        this->addTopDieInstItems();
        this->addExternalPortItems();
        this->placeExternalPortsByConnections();
        this->syncExportPortGroups();
        this->addNetItems();
        this->refreshPowerRails();
        this->markBundleNets();
    }

    void SchematicScene::addTopDieInstItems() {
        // Load all topdie inst!
        for (auto& [name, topdie] : this->_basedie->topdie_insts()) {
            auto t = this->addTopDieInst(topdie.get());
        }

        // Modest spacing between instances (wires may cross bodies).
        constexpr int kSpacingGrids = 3;
        constexpr qreal kInitialTopDieGapH = 8. * schematic::GridItem::GRID_SIZE; // 160
        constexpr qreal kInitialTopDieGapV = 8. * schematic::GridItem::GRID_SIZE; // 160

        int cols = std::ceil(std::sqrt(this->_topdieinstMap.size()));
        if (cols < 1) {
            cols = 1;
        }

        const qreal startX = kInitialTopDieGapH;
        const qreal startY = kInitialTopDieGapV;

        auto i = 0;
        for (auto& topdieInstItems : this->_topdieinstMap) {
            int row = i / cols;
            int col = i % cols;

            int x = startX + col * (topdieInstItems->width() + kInitialTopDieGapH);
            int y = startY + row * (topdieInstItems->height() + kInitialTopDieGapV);

            topdieInstItems->setPos(x, y);

            i += 1;
        }
    }

    void SchematicScene::addExternalPortItems() {
        for (auto& [name, eport] : this->_basedie->external_ports()) {
            this->addExPort(eport.get());
        }
    }

    void SchematicScene::placeExternalPortsByConnections() {
        // Place exports left/right of the topdie array so wires leave toward the array
        // without crossing the pin name: Left side uses PinSide::Right (name left of
        // origin); Right side uses PinSide::Left (name right of origin).
        QHash<circuit::ExternalPort*, qreal> targetY;
        QHash<circuit::ExternalPort*, qreal> targetX;
        QHash<circuit::ExternalPort*, int> targetCount;

        auto bumpPinPos = [this](const circuit::Pin& pin) -> std::optional<QPointF> {
            if (!pin.is_bump()) {
                return std::nullopt;
            }
            const auto& bump = pin.to_connect_bump();
            auto* top = this->_topdieinstMap.value(bump.inst, nullptr);
            if (!top) {
                return std::nullopt;
            }
            auto* pinItem = top->pins().value(QString::fromStdString(bump.name), nullptr);
            if (!pinItem) {
                return std::nullopt;
            }
            return pinItem->scenePos();
        };

        for (auto& [mode, inner] : this->_basedie->connections()) {
            for (auto& [sync, connections] : inner) {
                for (const auto& connection : connections) {
                    const auto& in = connection->input_pin();
                    const auto& out = connection->output_pin();

                    if (in.is_external_port()) {
                        if (auto p = bumpPinPos(out)) {
                            auto* port = in.to_connect_export().port;
                            targetX[port] += p->x();
                            targetY[port] += p->y();
                            targetCount[port] += 1;
                        }
                    }
                    if (out.is_external_port()) {
                        if (auto p = bumpPinPos(in)) {
                            auto* port = out.to_connect_export().port;
                            targetX[port] += p->x();
                            targetY[port] += p->y();
                            targetCount[port] += 1;
                        }
                    }
                }
            }
        }

        QRectF arrayBounds;
        bool hasTop = false;
        for (auto* item : this->_topdieinstMap) {
            const auto r = item->sceneBoundingRect();
            arrayBounds = hasTop ? arrayBounds.united(r) : r;
            hasTop = true;
        }
        const qreal arrayCenterX = hasTop ? arrayBounds.center().x() : 0.;
        const qreal gap = 6 * schematic::GridItem::GRID_SIZE;

        const qreal leftX = (hasTop ? arrayBounds.left() : 0.) - gap;
        const qreal rightX = (hasTop ? arrayBounds.right() : 0.) + gap;

        struct Place {
            schematic::ExternalPortItem* item;
            qreal y;
            bool onRight;
        };
        std::vector<Place> leftPlaces;
        std::vector<Place> rightPlaces;

        int fallbackIndex = 0;
        for (auto it = this->_exportMap.begin(); it != this->_exportMap.end(); ++it) {
            auto* port = it.key();
            auto* item = it.value();
            qreal y = 0;
            qreal xAvg = arrayCenterX;
            if (targetCount.value(port, 0) > 0) {
                y = targetY.value(port) / targetCount.value(port);
                xAvg = targetX.value(port) / targetCount.value(port);
            } else {
                y = fallbackIndex
                    * (schematic::ExternalPortItem::HEIGHT + schematic::GridItem::GRID_SIZE);
                ++fallbackIndex;
            }
            // Prefer the side closer to the connected bump; default right when equal
            // so PinSide::Left name (to the right of origin) stays clear of the wire.
            const bool onRight = xAvg >= arrayCenterX;
            (onRight ? rightPlaces : leftPlaces).push_back(Place{item, y, onRight});
        }

        auto packColumn = [](std::vector<Place>& places, qreal x, schematic::PinSide side) {
            std::sort(places.begin(), places.end(), [](const Place& a, const Place& b) {
                return a.y < b.y;
            });
            const qreal minGap =
                schematic::ExternalPortItem::HEIGHT + schematic::GridItem::GRID_SIZE;
            qreal lastY = -std::numeric_limits<qreal>::infinity();
            for (auto& place : places) {
                if (place.y < lastY + minGap) {
                    place.y = lastY + minGap;
                }
                place.item->setAnchorSide(side);
                place.item->setPos(QPointF{x, place.y});
                lastY = place.y;
            }
        };

        packColumn(leftPlaces, leftX, schematic::PinSide::Right);
        packColumn(rightPlaces, rightX, schematic::PinSide::Left);
    }

    void SchematicScene::syncExportPortGroups() {
        QVector<schematic::ExternalPortItem*> exports;
        exports.reserve(this->_exportMap.size());
        for (auto* item : this->_exportMap) {
            exports.push_back(item);
        }

        if (!this->_exportGroupHost) {
            this->_exportGroupHost = new schematic::ExportPortGroupHost{};
            this->addItem(this->_exportGroupHost);
        }
        this->_exportGroupHost->setExports(exports);
    }

    void SchematicScene::addNetItems() {
        for (auto& [mode, inner_connection]: this->_basedie->connections()) {
            for (auto& [sync, connections] : inner_connection) {
                for (const auto& connection : connections) {
                    auto* net = this->addNet(connection.get());
                    this->applyPowerNetPresentation(net);
                }
            }
        }
    }


    auto SchematicScene::isPowerConnection(const circuit::Connection* connection) -> bool {
        if (!connection) {
            return false;
        }
        return connection->input_pin().is_fixed() || connection->output_pin().is_fixed();
    }

    void SchematicScene::applyPowerNetPresentation(schematic::NetItem* net) {
        if (!net || net->isFloating() || !net->unwrap()) {
            return;
        }
        if (!isPowerConnection(net->unwrap())) {
            return;
        }

        // Ch.八: do not draw physical power nets like signals.
        net->setVisible(false);

        for (auto* pin : this->endpointPins(net)) {
            if (!pin) {
                continue;
            }
            if (auto* port = dynamic_cast<schematic::SourcePortItem*>(pin->parentItem())) {
                port->setVisible(false);
            }
        }
    }

    void SchematicScene::refreshPowerRails() {
        QSet<schematic::TopDieInstanceItem*> vddDies;
        QSet<schematic::TopDieInstanceItem*> gndDies;

        for (auto* net : this->_nets) {
            if (!net || net->isFloating() || !net->unwrap()) {
                continue;
            }
            if (!isPowerConnection(net->unwrap())) {
                continue;
            }

            const auto& in = net->unwrap()->input_pin();
            const auto& out = net->unwrap()->output_pin();
            const bool isVdd = in.is_vdd() || out.is_vdd();
            const bool isGnd = in.is_gnd() || out.is_gnd();

            for (auto* die : this->endpointTopDies(net)) {
                if (!die) {
                    continue;
                }
                if (isVdd) {
                    vddDies.insert(die);
                }
                if (isGnd) {
                    gndDies.insert(die);
                }
            }
        }

        QRectF arrayBounds;
        bool hasTop = false;
        for (auto* top : this->_topdieinstMap) {
            if (!top) {
                continue;
            }
            const auto r = top->sceneBoundingRect();
            arrayBounds = hasTop ? arrayBounds.united(r) : r;
            hasTop = true;
        }

        const qreal gap = 2.5 * schematic::GridItem::GRID_SIZE;
        const qreal pad = schematic::GridItem::GRID_SIZE;

        auto ensureRail = [this](schematic::PowerRailItem*& rail, schematic::PowerRailItem::Kind kind) {
            if (!rail) {
                rail = new schematic::PowerRailItem{kind};
                this->addItem(rail);
            }
        };

        auto layoutRail = [&](
            schematic::PowerRailItem*& rail,
            schematic::PowerRailItem::Kind kind,
            const QSet<schematic::TopDieInstanceItem*>& dies,
            bool isVdd
        ) {
            if (dies.isEmpty() || !hasTop) {
                if (rail) {
                    rail->setVisible(false);
                }
                return;
            }

            ensureRail(rail, kind);
            rail->setVisible(true);

            const qreal railY = isVdd
                ? arrayBounds.top() - gap
                : arrayBounds.bottom() + gap;
            const qreal x0 = arrayBounds.left() - pad;
            const qreal x1 = arrayBounds.right() + pad;

            QVector<QPointF> stubs;
            stubs.reserve(dies.size());
            for (auto* die : dies) {
                const auto r = die->sceneBoundingRect();
                const qreal x = r.center().x();
                const qreal y = isVdd ? r.top() : r.bottom();
                stubs.push_back(QPointF{x, y});
            }
            std::sort(stubs.begin(), stubs.end(), [](const QPointF& a, const QPointF& b) {
                return a.x() < b.x();
            });
            rail->setLayout(railY, x0, x1, stubs);
        };

        layoutRail(this->_vddRail, schematic::PowerRailItem::Kind::Vdd, vddDies, true);
        layoutRail(this->_gndRail, schematic::PowerRailItem::Kind::Gnd, gndDies, false);
    }

    void SchematicScene::refreshBusBundling() {
        this->markBundleNets();
    }

    void SchematicScene::markBundleNets() {
        // Reset non-power nets: clear prior collapse; power stays handled separately.
        for (auto* net : this->_nets) {
            if (!net || net->isFloating() || !net->unwrap()) {
                continue;
            }
            if (isPowerConnection(net->unwrap())) {
                continue;
            }
            net->setBundleMember(false);
            net->setBundleLabel(QString());
            net->setVisible(true);
        }

        const qreal s = this->viewScale();
        // Expand only by zoom to Near (Ch.九 9.0). Port Group size 0 (Far) must stay collapsed.
        const bool expand = s >= schematic::PinItem::LOD_NEAR_MIN;

        QHash<QPair<quintptr, quintptr>, QVector<schematic::NetItem*>> groups;
        for (auto* net : this->_nets) {
            if (!net || net->isFloating() || !net->unwrap()) {
                continue;
            }
            if (isPowerConnection(net->unwrap())) {
                continue;
            }
            const auto key = this->endpointPairKey(net);
            if (!key.has_value()) {
                continue;
            }
            groups[*key].push_back(net);
        }

        for (auto it = groups.begin(); it != groups.end(); ++it) {
            auto& members = it.value();
            if (members.size() < 2) {
                continue;
            }

            std::sort(members.begin(), members.end(), [](schematic::NetItem* a, schematic::NetItem* b) {
                return reinterpret_cast<quintptr>(a) < reinterpret_cast<quintptr>(b);
            });

            for (auto* net : members) {
                net->setBundleMember(true);
            }

            if (expand) {
                continue;
            }

            // Collapse: one thicker representative + count label; hide siblings.
            // Do not re-autoroute — keep the representative's existing path (Ch.九 hint).
            auto* rep = members.first();
            rep->setBundleLabel(QString::number(members.size()));
            for (int i = 1; i < members.size(); ++i) {
                members[i]->setVisible(false);
            }
        }

        // Re-apply default/focus widths so BUNDLE_WIDTH takes effect on visibles.
        this->refreshConnectionFocus();
    }

    auto SchematicScene::viewScale() const -> qreal {
        const auto vs = this->views();
        if (!vs.isEmpty() && vs.first()) {
            return vs.first()->transform().m11();
        }
        return 1.0;
    }

    auto SchematicScene::endpointPairKey(schematic::NetItem* net) const
        -> std::optional<QPair<quintptr, quintptr>>
    {
        if (!net || net->isFloating()) {
            return std::nullopt;
        }

        auto containerId = [](schematic::PinItem* pin) -> std::optional<quintptr> {
            if (!pin) {
                return std::nullopt;
            }
            if (pin->isTopDieInstancePin()) {
                if (auto* top = pin->parentTopDieInstance()) {
                    return reinterpret_cast<quintptr>(top);
                }
                return std::nullopt;
            }
            if (pin->isExternalPortPin()) {
                if (auto* eport = pin->parentExternalPort()) {
                    return reinterpret_cast<quintptr>(eport);
                }
                return std::nullopt;
            }
            // SourcePort (VDD/GND) — excluded from bus bundling.
            return std::nullopt;
        };

        schematic::PinItem* beginPin = nullptr;
        schematic::PinItem* endPin = nullptr;
        if (net->beginPoint()) {
            beginPin = net->beginPoint()->connectedPin();
        }
        if (net->endPoint()) {
            endPin = net->endPoint()->connectedPin();
        }

        const auto a = containerId(beginPin);
        const auto b = containerId(endPin);
        if (!a.has_value() || !b.has_value() || *a == *b) {
            return std::nullopt;
        }
        return (*a < *b) ? qMakePair(*a, *b) : qMakePair(*b, *a);
    }

    void SchematicScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
        if (this->_floatingNet != nullptr) {
            auto gridPos = schematic::GridItem::snapToGrid(event->scenePos());
            this->_floatingNet->updateEndPoint(gridPos);
        }
        if (this->_floatingTopdDieInst != nullptr) {
            auto gridPos = schematic::GridItem::snapToGrid(event->scenePos());
            this->_floatingTopdDieInst->setPos(gridPos);
        }
        if (this->_floatingExPort != nullptr) {
            auto gridPos = schematic::GridItem::snapToGrid(event->scenePos());
            this->_floatingExPort->setPos(gridPos);
        }
        QGraphicsScene::mouseMoveEvent(event);
    }

    void SchematicScene::emitSelectionForItem(QGraphicsItem* item) {
        if (!item) {
            this->_selectedFocusNets.clear();
            emit this->viewSelected();
            this->refreshConnectionFocus();
            return;
        }

        if (item->type() == schematic::NetItem::Type) {
            auto* net = dynamic_cast<schematic::NetItem*>(item);
            this->_selectedFocusNets = this->expandToBundleNets(net);
            emit this->netSelected(net);
            this->refreshConnectionFocus();
        }
        else if (item->type() == schematic::TopDieInstanceItem::Type) {
            auto* top = dynamic_cast<schematic::TopDieInstanceItem*>(item);
            if (top) {
                this->enforceSingleTopDieSelection(top);
                top->setSelected(true);
                this->_selectedTopDie = top;
                this->_selectedFocusNets.clear();
            }
            emit this->topdieInstSelected(top);
            this->refreshConnectionFocus();
        }
        else if (item->type() == schematic::ExternalPortItem::Type) {
            auto* eport = dynamic_cast<schematic::ExternalPortItem*>(item);
            if (eport) {
                eport->setSelected(true);
            }
            this->_selectedFocusNets.clear();
            emit this->exportSelected(eport);
            this->refreshConnectionFocus();
        }
        else if (item->type() == schematic::PortGroupItem::Type) {
            auto* group = dynamic_cast<schematic::PortGroupItem*>(item);
            if (group && group->ownerTopDie()) {
                this->enforceSingleTopDieSelection(group->ownerTopDie());
                group->ownerTopDie()->setSelected(true);
                this->_selectedTopDie = group->ownerTopDie();
                this->_selectedFocusNets = this->netsForPortGroup(group);
                emit this->topdieInstSelected(group->ownerTopDie());
            } else if (group && group->isExportGroup() && !group->exportMembers().isEmpty()) {
                for (auto* e : group->exportMembers()) {
                    if (e) {
                        e->setSelected(true);
                    }
                }
                this->_selectedFocusNets = this->netsForPortGroup(group);
                emit this->exportSelected(group->exportMembers().first());
            } else {
                this->_selectedFocusNets.clear();
                emit this->viewSelected();
            }
            this->refreshConnectionFocus();
        }
    }

    void SchematicScene::enforceSingleTopDieSelection(schematic::TopDieInstanceItem* keep) {
        for (auto* top : this->_topdieinstMap) {
            if (top && top != keep && top->isSelected()) {
                top->setSelected(false);
            }
        }
    }

    void SchematicScene::setHoverTopDie(schematic::TopDieInstanceItem* die) {
        if (this->_hoverTopDie == die) {
            return;
        }
        this->_hoverTopDie = die;
        this->refreshConnectionFocus();
    }

    void SchematicScene::setHoverPin(schematic::PinItem* pin) {
        QSet<schematic::NetItem*> nets;
        if (pin) {
            for (auto* point : pin->connectedPoints()) {
                if (point && point->netItem() && !point->netItem()->isFloating()) {
                    nets.unite(this->expandToBundleNets(point->netItem()));
                }
            }
        }
        if (nets == this->_hoverFocusNets) {
            return;
        }
        this->_hoverFocusNets = std::move(nets);
        this->refreshConnectionFocus();
    }

    void SchematicScene::setHoverNet(schematic::NetItem* net) {
        QSet<schematic::NetItem*> nets = this->expandToBundleNets(net);
        if (nets == this->_hoverFocusNets) {
            return;
        }
        this->_hoverFocusNets = std::move(nets);
        this->refreshConnectionFocus();
    }

    void SchematicScene::setHoverPortGroup(schematic::PortGroupItem* group) {
        QSet<schematic::NetItem*> nets = this->netsForPortGroup(group);
        if (nets == this->_hoverFocusNets) {
            return;
        }
        this->_hoverFocusNets = std::move(nets);
        this->refreshConnectionFocus();
    }

    void SchematicScene::onTopDieSelectionChanged(schematic::TopDieInstanceItem* die, bool selected) {
        if (this->_refreshingFocus) {
            return;
        }
        if (selected) {
            this->enforceSingleTopDieSelection(die);
            this->_selectedTopDie = die;
        } else if (this->_selectedTopDie == die) {
            this->_selectedTopDie = nullptr;
            // Another die may still be selected (rubber-band); pick one if present.
            for (auto* top : this->_topdieinstMap) {
                if (top && top->isSelected()) {
                    this->_selectedTopDie = top;
                    this->enforceSingleTopDieSelection(top);
                    break;
                }
            }
        }
        this->refreshConnectionFocus();
    }

    auto SchematicScene::endpointPins(schematic::NetItem* net) const -> QSet<schematic::PinItem*> {
        QSet<schematic::PinItem*> pins;
        if (!net) {
            return pins;
        }
        if (net->beginPoint() && net->beginPoint()->connectedPin()) {
            pins.insert(net->beginPoint()->connectedPin());
        }
        if (net->endPoint() && net->endPoint()->connectedPin()) {
            pins.insert(net->endPoint()->connectedPin());
        }
        return pins;
    }

    auto SchematicScene::endpointTopDies(schematic::NetItem* net) const -> QSet<schematic::TopDieInstanceItem*> {
        QSet<schematic::TopDieInstanceItem*> dies;
        for (auto* pin : this->endpointPins(net)) {
            if (pin && pin->isTopDieInstancePin()) {
                if (auto* top = pin->parentTopDieInstance()) {
                    dies.insert(top);
                }
            }
        }
        return dies;
    }

    auto SchematicScene::netsTouchingDie(schematic::TopDieInstanceItem* die) const -> QSet<schematic::NetItem*> {
        QSet<schematic::NetItem*> nets;
        if (!die) {
            return nets;
        }
        for (auto* net : this->_nets) {
            if (!net || net->isFloating()) {
                continue;
            }
            for (auto* pin : this->endpointPins(net)) {
                if (pin && pin->isTopDieInstancePin() && pin->parentTopDieInstance() == die) {
                    nets.insert(net);
                    break;
                }
            }
        }
        return nets;
    }

    auto SchematicScene::netsForPortGroup(schematic::PortGroupItem* group) const -> QSet<schematic::NetItem*> {
        QSet<schematic::NetItem*> nets;
        if (!group) {
            return nets;
        }
        if (group->isExportGroup()) {
            for (auto* eport : group->exportMembers()) {
                if (!eport || !eport->pin()) {
                    continue;
                }
                auto* pin = eport->pin();
                for (auto* point : pin->connectedPoints()) {
                    if (point && point->netItem() && !point->netItem()->isFloating()) {
                        nets.unite(this->expandToBundleNets(point->netItem()));
                    }
                }
            }
            return nets;
        }
        for (auto* pin : group->pinMembers()) {
            if (!pin) {
                continue;
            }
            for (auto* point : pin->connectedPoints()) {
                if (point && point->netItem() && !point->netItem()->isFloating()) {
                    nets.unite(this->expandToBundleNets(point->netItem()));
                }
            }
        }
        return nets;
    }

    auto SchematicScene::expandToBundleNets(schematic::NetItem* seed) const -> QSet<schematic::NetItem*> {
        QSet<schematic::NetItem*> out;
        if (!seed || seed->isFloating()) {
            return out;
        }
        out.insert(seed);

        // Ch.九: focus expands to the full endpoint-pair bundle (incl. hidden members).
        const auto key = this->endpointPairKey(seed);
        if (!key.has_value()) {
            return out;
        }
        for (auto* net : this->_nets) {
            if (!net || net->isFloating() || !net->unwrap()) {
                continue;
            }
            if (isPowerConnection(net->unwrap())) {
                continue;
            }
            const auto other = this->endpointPairKey(net);
            if (other.has_value() && *other == *key) {
                out.insert(net);
            }
        }
        return out;
    }

    void SchematicScene::refreshConnectionFocus() {
        if (this->_refreshingFocus) {
            return;
        }
        this->_refreshingFocus = true;

        // Priority: net/bundle focus > die focus > default.
        // Hover temporarily overrides selected when both are set.
        const QSet<schematic::NetItem*> focusNets =
            !this->_hoverFocusNets.isEmpty() ? this->_hoverFocusNets : this->_selectedFocusNets;
        schematic::TopDieInstanceItem* focusDie =
            this->_hoverTopDie ? this->_hoverTopDie : this->_selectedTopDie;

        const bool netFocus = !focusNets.isEmpty();
        const bool dieFocus = !netFocus && focusDie != nullptr;

        QSet<schematic::NetItem*> dieRelated;
        if (dieFocus) {
            dieRelated = this->netsTouchingDie(focusDie);
        }

        QSet<schematic::TopDieInstanceItem*> highlightDies;
        QSet<schematic::PinItem*> highlightPins;
        if (netFocus) {
            for (auto* net : focusNets) {
                highlightDies.unite(this->endpointTopDies(net));
                highlightPins.unite(this->endpointPins(net));
            }
        } else if (dieFocus) {
            highlightDies.insert(focusDie);
            for (auto* net : dieRelated) {
                highlightDies.unite(this->endpointTopDies(net));
                highlightPins.unite(this->endpointPins(net));
            }
        }

        for (auto* net : this->_nets) {
            if (!net || net->isFloating() || !net->isVisible()) {
                continue;
            }
            if (netFocus) {
                net->applyFocusRole(
                    focusNets.contains(net)
                        ? schematic::NetFocusRole::NetFocused
                        : schematic::NetFocusRole::NetUnrelated
                );
            } else if (dieFocus) {
                net->applyFocusRole(
                    dieRelated.contains(net)
                        ? schematic::NetFocusRole::DieRelated
                        : schematic::NetFocusRole::DieUnrelated
                );
            } else {
                net->applyFocusRole(schematic::NetFocusRole::Default);
            }
        }

        for (auto* top : this->_topdieinstMap) {
            if (top) {
                top->setFocusBorder(highlightDies.contains(top));
            }
        }

        // Force-show related pins (Ch.五 LOD override via Ch.七 focus).
        for (auto* top : this->_topdieinstMap) {
            if (!top) {
                continue;
            }
            for (auto* pin : top->pins()) {
                if (pin) {
                    pin->setFocusRelated(highlightPins.contains(pin));
                }
            }
        }
        for (auto* eport : this->_exportMap) {
            if (!eport || !eport->pin()) {
                continue;
            }
            auto* pin = eport->pin();
            pin->setFocusRelated(highlightPins.contains(pin));
        }

        this->_refreshingFocus = false;
    }

    void SchematicScene::mousePressEvent(QGraphicsSceneMouseEvent* event) {
        const bool wasFloating = this->_floatingNet != nullptr
            || this->_floatingTopdDieInst != nullptr
            || this->_floatingExPort != nullptr;

        if (this->_floatingNet != nullptr) {
            if (event->button() & Qt::LeftButton) {
                auto gridPos = schematic::GridItem::snapToGrid(event->scenePos());
                this->_floatingNet->addPoint(gridPos);
            }
            else if (event->button() & Qt::RightButton) {
                this->cancelFloatingPlacement();
            }
        }

        if (this->_floatingTopdDieInst != nullptr) {
            if (event->button() & Qt::LeftButton) {
                this->placeFloatingTopdDieInst();
            }
            else if (event->button() & Qt::RightButton) {
                this->cancelFloatingPlacement();
            }
        }
        
        if (this->_floatingExPort != nullptr) {
            if (event->button() & Qt::LeftButton) {
                this->placeFloatingExPort();
            }
            else if (event->button() & Qt::RightButton) {
                this->cancelFloatingPlacement();
            }
        }

        QGraphicsScene::mousePressEvent(event);

        // Single-click drives the property panel the same way double-click does (U7/S9).
        // Skip while placing/wiring so pin-start and floating placement are undisturbed.
        const bool nowFloating = this->_floatingNet != nullptr
            || this->_floatingTopdDieInst != nullptr
            || this->_floatingExPort != nullptr;
        if (!wasFloating && !nowFloating && (event->button() & Qt::LeftButton)) {
            this->emitSelectionForItem(this->itemAt(event->scenePos(), QTransform()));
        }
    }

    void SchematicScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
        this->emitSelectionForItem(this->itemAt(event->scenePos(), QTransform()));
        QGraphicsScene::mouseDoubleClickEvent(event);
    }

    auto SchematicScene::addNetPoint(schematic::PinItem* pin) -> schematic::NetPointItem* {
        auto point = new schematic::NetPointItem {pin};
        this->addItem(point);
        return point;
    }

    auto SchematicScene::addExPort(circuit::ExternalPort* eport) -> schematic::ExternalPortItem* {
        auto item = new schematic::ExternalPortItem {eport};
        this->_exportMap.insert(eport, item);
        this->addItem(item);
        return item;
    }

    namespace {

        auto maxTopDiePinCount(circuit::BaseDie* basedie) -> std::size_t {
            std::size_t maxPins = 0;
            for (const auto& [_, topdie] : basedie->topdies()) {
                maxPins = std::max(maxPins, topdie->pins_map().size());
            }
            return maxPins;
        }

    }

    auto SchematicScene::addTopDieInst(circuit::TopDieInstance* inst) -> schematic::TopDieInstanceItem* {
        auto item = new schematic::TopDieInstanceItem{inst, maxTopDiePinCount(this->_basedie)};
        this->_topdieinstMap.insert(inst, item);
        this->addItem(item);
        return item;
    }

    auto SchematicScene::addNet(circuit::Connection* connection) -> schematic::NetItem* {
        auto beginPin = this->circuitPinToPinItem(connection->input_pin());
        auto endPin = this->circuitPinToPinItem(connection->output_pin());

        auto beginPoint = this->addNetPoint(beginPin);
        auto endPoint = this->addNetPoint(endPin);

        auto item = new schematic::NetItem {connection, beginPoint, endPoint};
        this->addItem(item);
        this->_nets.insert(item);

        return item;
    }

    auto SchematicScene::addVDDSourcePort(const QString& name) -> schematic::SourcePortItem* {
        auto item = new schematic::SourcePortItem{name, schematic::SourcePortType::VDD};
        this->_vddPorts.push_back(item);
        this->addItem(item);
        
        auto x = 0.;
        for (auto i : this->_vddPorts) {
            x += (i->width() + 2 * schematic::GridItem::GRID_SIZE);
        }

        item->setPos(QPointF{x, -5 * schematic::GridItem::GRID_SIZE});

        return item;
    }

    auto SchematicScene::addGNDSourcePort(const QString& name) -> schematic::SourcePortItem* {
        auto item = new schematic::SourcePortItem{name, schematic::SourcePortType::GND};
        this->_gndPorts.push_back(item);
        this->addItem(item);
        
        auto x = 0.;
        for (auto i : this->_gndPorts) {
            x += (i->width() + 2 * schematic::GridItem::GRID_SIZE);
        }

        item->setPos(QPointF{x, -10 * schematic::GridItem::GRID_SIZE});

        return item;
    }

    void SchematicScene::removeExternalPort(schematic::ExternalPortItem* eport) {
        assert(eport != nullptr);
        // debug::debug_fmt("Remove external port '{}'", eport->exPort()->name());

        auto pinItem = eport->pin();
        assert(pinItem != nullptr);
        
        for (auto pointItem : pinItem->connectedPoints()) {
            auto netItem = pointItem->netItem();
            assert(netItem != nullptr);
            this->removeNet(netItem);
        }

        delete pinItem;

        this->_exportMap.remove(eport->unwrap());
        this->removeItem(eport);

        delete eport;
        this->syncExportPortGroups();
    }

    void SchematicScene::removeTopDieInstance(schematic::TopDieInstanceItem* inst) {
        assert(inst != nullptr);
        // debug::debug_fmt("Remove topdie instance '{}'", inst->topdieInst()->name());

        for (auto pinItem : inst->pins()) {
            for (auto point : pinItem->connectedPoints()) {
                assert(point != nullptr);
                assert(point->netItem() != nullptr);
                this->removeNet(point->netItem());
            }
            delete pinItem;
        }

        this->_topdieinstMap.remove(inst->unwrap());
        this->removeItem(inst);

        delete inst;
    }

    void SchematicScene::removeNet(schematic::NetItem* net) {
        // net has two point, each pin has 
        assert(net != nullptr);
        // debug::debug_fmt("Remove net '{}'", *net->connection());
    
        auto beginPoint = net->beginPoint();
        auto endPoint = net->endPoint();

        assert(beginPoint != nullptr && endPoint != nullptr);

        auto beginPin = beginPoint->connectedPin();
        auto endPin = endPoint->connectedPin();

        assert(beginPoint != nullptr && endPoint != nullptr);

        beginPin->removeConnectedPoint(beginPoint);
        endPin->removeConnectedPoint(endPoint);

        this->removeItem(net);
        this->_nets.remove(net);

        delete net;
        delete beginPoint;
        delete endPoint;

        this->refreshPowerRails();
        this->markBundleNets();
    }

    auto SchematicScene::circuitPinToPinItem(const circuit::Pin& pin) -> schematic::PinItem* {
        return std::match(pin.connected_point(),
            [this](const circuit::ConnectVDD& vdd) -> schematic::PinItem* {
                return this->addVDDSourcePort(QString::fromStdString(vdd.name))->pin();
            },
            [this](const circuit::ConnectGND& gnd) -> schematic::PinItem* {
                return this->addGNDSourcePort(QString::fromStdString(gnd.name))->pin();
            },
            [this](const circuit::ConnectExPort& eport) -> schematic::PinItem* {
                auto eportItem = this->_exportMap.value(eport.port);
                return eportItem->pin();
            },
            [this](const circuit::ConnectBump& bump) -> schematic::PinItem* {
                auto top = this->_topdieinstMap.value(bump.inst);
                return top->pins().value(QString::fromStdString(bump.name));
            }
        );
    }

    void SchematicScene::headleCreateNet(schematic::PinItem* pin, QGraphicsSceneMouseEvent* event) {
        if (this->_floatingNet != nullptr) {
            auto* beginPin = this->_floatingNet->beginPoint()->connectedPin();

            if (pin == beginPin) {
                QMessageBox::warning(
                    nullptr,
                    "Create Net Error",
                    "Cannot connect a pin to itself."
                );
                return;
            }

            if (!pin->connectedPoints().isEmpty()) {
                QMessageBox::warning(
                    nullptr,
                    "Create Net Error",
                    "End pin already has a connection."
                );
                return;
            }

            auto endPoint = this->addNetPoint(pin);
            
            auto connection = this->_basedie->add_connection(
                0,
                -1, 
                beginPin->toCircuitPin(),
                endPoint->connectedPin()->toCircuitPin()
            );

            this->_floatingNet->wrap(connection);
            this->_floatingNet->setEndPoint(endPoint);

            this->_floatingNet->resetPaint();
            endPoint->setNetItem(this->_floatingNet);
            this->_nets.insert(this->_floatingNet);
            this->applyPowerNetPresentation(this->_floatingNet);
            this->refreshPowerRails();
            this->markBundleNets();

            this->_floatingNet = nullptr;

            emit this->layoutChanged();
        } 
        else {
            auto beginPoint = this->addNetPoint(pin);
            
            this->_floatingNet = new schematic::NetItem {beginPoint};
            this->addItem(this->_floatingNet);
        }
    }

    void SchematicScene::adjustSceneRect() {
        for (QGraphicsView* view : this->views()) {
            if (auto* gv = qobject_cast<GraphicsView*>(view)) {
                gv->adjustSceneRect();
            }
        }
    }

    void SchematicScene::handleInitialTopDie(circuit::TopDie* topdie) {
        auto idle_tob = this->_interposer->get_a_idle_tob();
        if (!idle_tob.has_value()) {
            QMessageBox::critical(
                nullptr,
                "Add TopDies Error",
                "No idle tob to place topdie inst!"
            );
        } 
        else {
            auto topdieInst = this->_basedie->add_topdie_inst(topdie, *idle_tob);
            auto topdieInstItem = this->addTopDieInst(topdieInst);

            if (this->_floatingTopdDieInst != nullptr) {
                this->cleanFloatingTopdDieInst();
            }

            this->_floatingTopdDieInst = topdieInstItem;

            this->adjustSceneRect();
            emit this->layoutChanged();
        }
    }

    void SchematicScene::handleAddVdd() {
        const auto name = QStringLiteral("VDD_%1").arg(this->_vddPorts.size());
        this->addVDDSourcePort(name);
        this->adjustSceneRect();
    }

    void SchematicScene::handleAddGnd() {
        const auto name = QStringLiteral("GND_%1").arg(this->_gndPorts.size());
        this->addGNDSourcePort(name);
        this->adjustSceneRect();
    }

    void SchematicScene::handleAddExport() {
        auto eport = this->_basedie->add_external_port({});
        auto eportItem = this->addExPort(eport);

        if (this->_floatingExPort != nullptr) {
            this->cleanFloatingExPort();
        }

        this->_floatingExPort = eportItem;

        this->adjustSceneRect();
        emit this->layoutChanged();
    }

    void SchematicScene::cancelFloatingPlacement() {
        if (this->_floatingNet != nullptr) {
            this->cleanFloatingNet();
        }
        if (this->_floatingTopdDieInst != nullptr) {
            this->cleanFloatingTopdDieInst();
        }
        if (this->_floatingExPort != nullptr) {
            this->cleanFloatingExPort();
        }
    }

    void SchematicScene::placeFloatingTopdDieInst() {
        assert(this->_floatingTopdDieInst != nullptr);
        this->_floatingTopdDieInst = nullptr;
        this->adjustSceneRect();
    }

    void SchematicScene::cleanFloatingTopdDieInst() {
        assert(this->_floatingTopdDieInst != nullptr);
        assert(this->_floatingTopdDieInst->unwrap() != nullptr);

        this->_basedie->remove_topdie_inst(this->_floatingTopdDieInst->unwrap());
        this->removeItem(this->_floatingTopdDieInst);
        this->_floatingTopdDieInst = nullptr;

        emit this->layoutChanged();
    }

    void SchematicScene::placeFloatingExPort() {
        assert(this->_floatingExPort != nullptr);
        this->_floatingExPort = nullptr;
        this->syncExportPortGroups();
        this->adjustSceneRect();
    }

    void SchematicScene::cleanFloatingExPort() {
        assert(this->_floatingExPort != nullptr);
        assert(this->_floatingExPort->unwrap() != nullptr);

        this->_basedie->remove_external_port(this->_floatingExPort->unwrap());
        this->removeItem(this->_floatingExPort);
        this->_floatingExPort = nullptr;

        emit this->layoutChanged();
    }

    void SchematicScene::cleanFloatingNet() {
        assert(this->_floatingNet != nullptr);

        auto beginPoint = this->_floatingNet->beginPoint();
        assert(beginPoint != nullptr);

        beginPoint->unlinkPin();
        this->removeItem(this->_floatingNet);
        this->removeItem(beginPoint);

        delete this->_floatingNet;
        delete beginPoint;
        this->_floatingNet = nullptr;
    }

}