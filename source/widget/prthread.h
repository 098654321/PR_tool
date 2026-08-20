#pragma once

#include <QString>
#include <QStringList>
#include <QThread>
#include <debug/debug.hh>
#include <algo/route_data.hh>
#include <algo/router/route_nets.hh>
#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/placer/place.hh>
#include <algo/placer/sa/saplacestrategy.hh>
#include <parse/writer/module.hh>
#include <circuit/basedie.hh>
#include <circuit/topdieinst/topdieinst.hh>

#include <exception>

namespace PR_tool::widget {

    class PRThread : public QThread {
        Q_OBJECT

    public:
        PRThread(hardware::Interposer* i, circuit::BaseDie* b) : 
            QThread{},
            _interposer{i},
            _basedie{b} 
        {
        }

    protected:
        void run() override {
            try {
                debug::info_fmt("Begin to execute P&R");
                if (this->_basedie->nets().empty()) {
                    algo::build_nets(this->_basedie, this->_interposer);
                }
                auto result = algo::route_nets(
                    this->_interposer,
                    this->_basedie,
                    algo::MazeRouteStrategy{},
                    algo::HK{},
                    0,
                    false,
                    false,
                    false,
                    [this](std::usize done, std::usize total) {
                        emit this->routeProgress(
                            static_cast<int>(done),
                            static_cast<int>(total));
                    });

                if (!result.failed_net_names.empty()) {
                    QStringList names;
                    constexpr int kMaxNames = 20;
                    const int total = static_cast<int>(result.failed_net_names.size());
                    for (int i = 0; i < total && i < kMaxNames; ++i) {
                        names << QString::fromStdString(result.failed_net_names[static_cast<std::size_t>(i)]);
                    }
                    QString message = QStringLiteral("Routing failed: %1 net(s) could not be routed: %2")
                        .arg(total)
                        .arg(names.join(QStringLiteral(", ")));
                    if (total > kMaxNames) {
                        message += QStringLiteral(" ...");
                    }
                    debug::info_fmt("P&R failed with {} failed net(s)", total);
                    emit this->prFinished(false, message);
                    return;
                }

                parse::connect_registers(this->_interposer, this->_basedie, 0);
                debug::info_fmt("P&R finished with total path length '{}'", result.data._total_length);
                emit this->prFinished(true, QString{});
            } catch (const std::exception& e) {
                debug::info_fmt("P&R failed with exception: {}", e.what());
                emit this->prFinished(
                    false,
                    QStringLiteral("Routing failed: %1").arg(QString::fromUtf8(e.what()))
                );
            } catch (...) {
                debug::info_fmt("P&R failed with unknown exception");
                emit this->prFinished(false, QStringLiteral("Routing failed: unknown error"));
            }
        }

        signals:
        void prFinished(bool success, QString message);
        void routeProgress(int done, int total);

    protected:
        hardware::Interposer* _interposer;
        circuit::BaseDie* _basedie;
    };

    class PlaceThread : public QThread {
        Q_OBJECT

    public:
        PlaceThread(hardware::Interposer* i, circuit::BaseDie* b) :
            QThread{},
            _interposer{i},
            _basedie{b}
        {
        }

    protected:
        void run() override {
            try {
                debug::info_fmt("Begin to execute placement");
                if (this->_basedie->nets().empty()) {
                    algo::build_nets(this->_basedie, this->_interposer);
                }
                std::Vector<circuit::TopDieInstance*> topdies;
                for (auto& [name, inst] : this->_basedie->topdie_insts()) {
                    (void)name;
                    topdies.push_back(inst.get());
                }
                if (topdies.empty()) {
                    emit this->placeFinished(false, QStringLiteral("No chip instances to place"));
                    return;
                }
                auto strategy = algo::SAPlaceStrategy{};
                strategy.set_progress_callback(
                    [this](std::size_t iteration, std::i64 cost) {
                        emit this->placeProgress(
                            static_cast<int>(iteration),
                            static_cast<qint64>(cost));
                    });
                algo::place(this->_interposer, topdies, this->_basedie, strategy);
                if (!strategy.is_valid_placement(this->_interposer, topdies)) {
                    emit this->placeFinished(false, QStringLiteral("Placement is not valid"));
                    return;
                }
                debug::info_fmt("Placement finished");
                emit this->placeFinished(true, QString{});
            } catch (const std::exception& e) {
                debug::info_fmt("Placement failed with exception: {}", e.what());
                emit this->placeFinished(
                    false,
                    QStringLiteral("Placement failed: %1").arg(QString::fromUtf8(e.what()))
                );
            } catch (...) {
                emit this->placeFinished(false, QStringLiteral("Placement failed: unknown error"));
            }
        }

        signals:
        void placeFinished(bool success, QString message);
        void placeProgress(int iteration, qint64 cost);

    protected:
        hardware::Interposer* _interposer;
        circuit::BaseDie* _basedie;
    };

}
