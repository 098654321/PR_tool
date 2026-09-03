#include "./schematicinfowidget.h"

#include "./info/exportwidget.h"
#include "./info/netinfowidget.h"
#include "./info/tpdinfowidget.h"
#include "./info/viewinfowidget.h"
#include "./item/exportitem.h"
#include "./item/netitem.h"
#include "./item/topdieinstitem.h"
#include "./item/topdieinstitem.h"
#include "./schematicscene.h"

#include <std/exception.hh>
#include <widget/frame/msgexception.h>

#include <debug/debug.hh>
#include <circuit/basedie.hh>

#include <QSplitter>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QStackedWidget>
#include <QMessageBox>

namespace PR_tool::widget {

    using namespace schematic;

    SchematicInfoWidget::SchematicInfoWidget(
        circuit::BaseDie* basedie, 
        SchematicScene* scene,
        SchematicView* view, 
        QWidget* parent
    ) :
        QStackedWidget{parent},
        _basedie{basedie},
        _scene{scene},
        _view{view}
    {
        this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        this->createViewInfoWidget();
        this->createNetInfoWidget();
        this->createTopDieInstanceInfoWidget();
        this->createExternalPortInfoWidget();
    }

    void SchematicInfoWidget::reload() {
        this->setCurrentWidget(this->_viewInfoWidget);
    }

    void SchematicInfoWidget::createViewInfoWidget() {
        this->_viewInfoWidget = new ViewInfoWidget {this->_view, this};
        this->addWidget(this->_viewInfoWidget);
    }

    void SchematicInfoWidget::createExternalPortInfoWidget() {
        this->_eportInfoWidget = new ExternalPortInfoWidget {this};
        this->addWidget(this->_eportInfoWidget);

        connect(this->_eportInfoWidget, &ExternalPortInfoWidget::externalPortRename, this, &SchematicInfoWidget::externalPortRename);
        connect(this->_eportInfoWidget, &ExternalPortInfoWidget::externalPortSetCoord, this, &SchematicInfoWidget::externalPortSetCoord);
        connect(this->_eportInfoWidget, &ExternalPortInfoWidget::removeExternalPort, this,&SchematicInfoWidget::removeExternalPort);
    }

    void SchematicInfoWidget::createNetInfoWidget() {
        this->_netInfoWidget = new NetInfoWidget {this};
        this->addWidget(this->_netInfoWidget);

        connect(this->_netInfoWidget, &NetInfoWidget::netSyncChanged, this,&SchematicInfoWidget::netSyncChanged);
        connect(this->_netInfoWidget, &NetInfoWidget::netColorChanged, this,&SchematicInfoWidget::netColorChanged);
        connect(this->_netInfoWidget, &NetInfoWidget::netWidthChanged, this,&SchematicInfoWidget::netWidthChanged);
        connect(this->_netInfoWidget, &NetInfoWidget::removeNet, this,&SchematicInfoWidget::removeNet);
    }

    void SchematicInfoWidget::createTopDieInstanceInfoWidget() {
        this->_topdieInstInfoWidget = new TopDieInstanceInfoWidget {this->_scene, this};
        this->addWidget(this->_topdieInstInfoWidget);

        connect(this->_topdieInstInfoWidget, &TopDieInstanceInfoWidget::topdieInstanceRename, this,&SchematicInfoWidget::topdieInstanceRename);
        connect(this->_topdieInstInfoWidget, &TopDieInstanceInfoWidget::removeTopDieInstance, this,&SchematicInfoWidget::removeTopDieInstance);
    }

    void SchematicInfoWidget::showExPortInfoWidget(ExternalPortItem* eport) {
        this->setCurrentWidget(this->_eportInfoWidget);
        this->_eportInfoWidget->loadExternalPort(eport);
    }

    void SchematicInfoWidget::showNetInfoWidget(NetItem* net) {
        this->setCurrentWidget(this->_netInfoWidget);
        this->_netInfoWidget->loadNet(net);
    }

    void SchematicInfoWidget::showTopDieInstanceInfoWidget(TopDieInstanceItem* inst) {
        const bool alreadyShown =
            this->currentWidget() == this->_topdieInstInfoWidget
            && this->_topdieInstInfoWidget->currentTopDieInstance() == inst;
        this->setCurrentWidget(this->_topdieInstInfoWidget);
        if (alreadyShown) {
            return;
        }
        this->_topdieInstInfoWidget->loadTopDieInstance(inst);
    }

    void SchematicInfoWidget::showViewInfo() {
        this->setCurrentWidget(this->_viewInfoWidget);
    }

    void SchematicInfoWidget::deleteCurrentItem() {
        auto* current = this->currentWidget();
        if (current == this->_netInfoWidget) {
            auto* net = this->_netInfoWidget->currentNet();
            if (net == nullptr) {
                return;
            }
            auto response = QMessageBox::question(
                this,
                "Confirm",
                "Do you want to delete this net?",
                QMessageBox::Yes | QMessageBox::No);
            if (response == QMessageBox::Yes) {
                this->removeNet(net);
            }
        }
        else if (current == this->_eportInfoWidget) {
            auto* eport = this->_eportInfoWidget->currentExternalPort();
            if (eport == nullptr) {
                return;
            }
            auto response = QMessageBox::question(
                this,
                "Confirm",
                "Do you want to delete this external port?",
                QMessageBox::Yes | QMessageBox::No);
            if (response == QMessageBox::Yes) {
                this->removeExternalPort(eport);
            }
        }
        else if (current == this->_topdieInstInfoWidget) {
            auto* inst = this->_topdieInstInfoWidget->currentTopDieInstance();
            if (inst == nullptr) {
                return;
            }
            auto response = QMessageBox::question(
                this,
                "Confirm",
                "Do you want to delete this topdie instance?",
                QMessageBox::Yes | QMessageBox::No);
            if (response == QMessageBox::Yes) {
                this->removeTopDieInstance(inst);
            }
        }
    }

    void SchematicInfoWidget::externalPortRename(ExternalPortItem* eport, const QString& name) try {
        this->_basedie->external_port_rename(eport->unwrap(), name.toStdString());
        eport->setName(name);
        emit this->layoutChanged();
    } 
    QMESSAGEBOX_REPORT_EXCEPTION("External Port Rename Failed") 

    void SchematicInfoWidget::externalPortSetCoord(ExternalPortItem* eport, const hardware::TrackCoord& coord) try {
        this->_basedie->external_port_set_coord(eport->unwrap(), coord);
        emit this->layoutChanged();
    } 
    QMESSAGEBOX_REPORT_EXCEPTION("External Port Set Coord Failed")
    
    void SchematicInfoWidget::removeExternalPort(ExternalPortItem* eport) try {
        this->_basedie->remove_external_port(eport->unwrap());
        this->_scene->removeExternalPort(eport);
        this->showViewInfo();
        emit this->layoutChanged();
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Remove External Port Coord Failed")

    void SchematicInfoWidget::netSyncChanged(NetItem* net, int sync) try {
        this->_basedie->connection_set_sync(net->unwrap(), sync);
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Net Sync Change Failed")

    void SchematicInfoWidget::netColorChanged(NetItem* net, const QColor& color) {
        net->setColor(color);
        net->update();
    }

    void SchematicInfoWidget::netWidthChanged(NetItem* net, qreal width) {
        net->setWidth(width);
        net->update();
    }

    void SchematicInfoWidget::removeNet(NetItem* net) {
        this->_basedie->remove_connection(net->unwrap());
        this->_scene->removeNet(net);
        this->showViewInfo();
        emit this->layoutChanged();
    }

    void SchematicInfoWidget::topdieInstanceRename(TopDieInstanceItem* inst, const QString& name) try {
        this->_basedie->topdie_inst_rename(inst->unwrap(), name.toStdString());
        inst->setName(name);
        emit this->layoutChanged();
    }
    QMESSAGEBOX_REPORT_EXCEPTION("TopDie Instance Rename Failed")

    void SchematicInfoWidget::removeTopDieInstance(TopDieInstanceItem* inst) try {
        this->_basedie->remove_topdie_inst(inst->unwrap());
        this->_scene->removeTopDieInstance(inst);
        this->showViewInfo();
        emit this->layoutChanged();
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Remove TopDie Filed")

}