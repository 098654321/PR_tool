#pragma once

#include <std/memory.hh>
#include <QWidget>
#include <circuit/topdie/topdie.hh>

class QLineEdit;
class QVBoxLayout;

namespace PR_tool::circuit {
    class BaseDie;
};

namespace PR_tool::widget {

    class SchematicLibWidget : public QWidget {
        Q_OBJECT

    public:
        SchematicLibWidget(circuit::BaseDie* basedie, QWidget* parent = nullptr);

    public:
        void reload();

    signals:
        void initialTopDieInst(circuit::TopDie* topdie);
        void addExport();
        void addVdd();
        void addGnd();

    public:
        void onLoadTopDieClicked();
        void onLoadTopDiesClicked();

        void loadTopDie(const QString& path);
        void loadTopDies(const QString& path);
            
        void addTopDie(std::String name, std::HashMap<std::String, std::usize> pinmap);
        void addTopDie(circuit::TopDie* topdie);

    private:
        void loadTopDiesFromBaseDie();
        void applySearchFilter();

    private:
        QLineEdit* _searchEdit {nullptr};
        QVBoxLayout* _libraryLayout;

        circuit::BaseDie* _basedie {nullptr};
    };

}