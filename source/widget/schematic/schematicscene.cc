#include "./schematicscene.h"

#include "./item/netitem.h"
#include "./item/netpointitem.h"
#include "./item/pinitem.h"
#include "./item/exportitem.h"
#include "./item/topdieinstitem.h"
#include "./item/portgroupitem.h"
#include "./item/griditem.h"
#include "./item/sourceportitem.h"

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
#include <QTimer>

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
        (int)schematic::ExportPortGroupHost::Type
    >::value);

    SchematicScene::SchematicScene(circuit::BaseDie* basedie, hardware::Interposer* interposer) :
        _basedie{basedie},
        _interposer{interposer},
        QGraphicsScene{}
    {
        this->addSceneItems();
    }

    void SchematicScene::reloadItems() {
        // Clear
        this->_topdieinstMap.clear();
        this->_exportMap.clear();
        this->_nets.clear();
        this->_vddPorts.clear();
        this->_gndPorts.clear();

        this->_floatingNet = nullptr;
        this->_floatingTopdDieInst = nullptr;
        this->_floatingExPort = nullptr;
        this->_exportGroupHost = nullptr;
        this->_pendingTopDieGroupSync.clear();
        this->_pendingExportGroupSync = false;
        this->_portGroupFlushScheduled = false;

        this->clear();

        this->addSceneItems();
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
    }

    void SchematicScene::addSceneItems() {
        this->addTopDieInstItems();
        this->addExternalPortItems();
        this->placeExternalPortsByConnections();
        this->syncExportPortGroups();
        this->addNetItems();
    }

    void SchematicScene::addTopDieInstItems() {
        // Load all topdie inst!
        for (auto& [name, topdie] : this->_basedie->topdie_insts()) {
            auto t = this->addTopDieInst(topdie.get());
        }

        // Modest spacing between instances (wires may cross bodies).
        constexpr int kSpacingGrids = 3;
        auto spacing = kSpacingGrids * schematic::GridItem::GRID_SIZE;

        int cols = std::ceil(std::sqrt(this->_topdieinstMap.size()));
        if (cols < 1) {
            cols = 1;
        }

        int startX = spacing;
        int startY = spacing;

        auto i = 0;
        for (auto& topdieInstItems : this->_topdieinstMap) {
            int row = i / cols;
            int col = i % cols;

            int x = startX + col * (topdieInstItems->width() + spacing);
            int y = startY + row * (topdieInstItems->height() + spacing);

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
                    this->addNet(connection.get());
                }
            }
        }
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
            emit this->viewSelected();
            return;
        }

        if (item->type() == schematic::NetItem::Type) {
            emit this->netSelected(dynamic_cast<schematic::NetItem*>(item));
        }
        else if (item->type() == schematic::TopDieInstanceItem::Type) {
            auto* top = dynamic_cast<schematic::TopDieInstanceItem*>(item);
            if (top) {
                top->setSelected(true);
            }
            emit this->topdieInstSelected(top);
        }
        else if (item->type() == schematic::ExternalPortItem::Type) {
            auto* eport = dynamic_cast<schematic::ExternalPortItem*>(item);
            if (eport) {
                eport->setSelected(true);
            }
            emit this->exportSelected(eport);
        }
        else if (item->type() == schematic::PortGroupItem::Type) {
            auto* group = dynamic_cast<schematic::PortGroupItem*>(item);
            if (group && group->ownerTopDie()) {
                group->ownerTopDie()->setSelected(true);
                emit this->topdieInstSelected(group->ownerTopDie());
            } else if (group && group->isExportGroup() && !group->exportMembers().isEmpty()) {
                for (auto* e : group->exportMembers()) {
                    if (e) {
                        e->setSelected(true);
                    }
                }
                emit this->exportSelected(group->exportMembers().first());
            } else {
                emit this->viewSelected();
            }
        }
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