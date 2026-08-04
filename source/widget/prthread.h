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
#include <parse/writer/module.hh>

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
                algo::build_nets(this->_basedie, this->_interposer);
                auto result = algo::route_nets(this->_interposer, this->_basedie, algo::MazeRouteStrategy{}, algo::HK{}, 0, false, false);

                if (!result.failed_net_names.empty()) {
                    QStringList names;
                    constexpr int kMaxNames = 20;
                    const int total = static_cast<int>(result.failed_net_names.size());
                    for (int i = 0; i < total && i < kMaxNames; ++i) {
                        names << QString::fromStdString(result.failed_net_names[static_cast<std::size_t>(i)]);
                    }
                    QString message = QStringLiteral("P&R failed: %1 net(s) could not be routed: %2")
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
                    QStringLiteral("P&R failed: %1").arg(QString::fromUtf8(e.what()))
                );
            } catch (...) {
                debug::info_fmt("P&R failed with unknown exception");
                emit this->prFinished(false, QStringLiteral("P&R failed: unknown error"));
            }
        }

    signals:
        void prFinished(bool success, QString message);

    protected:
        hardware::Interposer* _interposer;
        circuit::BaseDie* _basedie;
    };

}
