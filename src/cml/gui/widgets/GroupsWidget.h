//
// Created by thomas on 06/12/2020.
//

#ifndef CML_GROUPSWIDGET_H
#define CML_GROUPSWIDGET_H

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QCheckBox>
#include <cml/base/AbstractSlam.h>

namespace CML {

    class GroupsWidget : public QWidget {

    Q_OBJECT

    public:
        GroupsWidget(AbstractSlam *slam, QWidget *parent = nullptr);

        ~GroupsWidget();

    signals:
        void onGroupsChanged(unsigned int groups);

    public slots:
        void updateGroups();

    private:
        QGridLayout mLayout;

        QLabel *mLabels[MAXGROUPSIZE];
        QCheckBox *mCheckboxs[MAXGROUPSIZE];

    };

}


#endif //CML_GROUPSWIDGET_H
