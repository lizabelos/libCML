//
// Created by thomas on 06/12/2020.
//

#include "cml/gui/widgets/GroupsWidget.h"

CML::GroupsWidget::GroupsWidget(AbstractSlam *slam, QWidget *parent) {

    for (int i = 0; i < MAXGROUPSIZE; i++) {
        mCheckboxs[i] = nullptr;
        mLabels[i] = nullptr;
    }

    for (int i = 0; i < slam->getMap().getMapPointsGroupsManager().getGroupNumber(); i++) {

        std::string name = slam->getMap().getMapPointsGroupsManager().getName(i);

        mCheckboxs[i] = new QCheckBox();
        mLayout.addWidget(mCheckboxs[i], i, 0);

        mLabels[i] = new QLabel();
        mLabels[i]->setText(QString::fromStdString(name));

        mLayout.addWidget(mLabels[i], i, 1);

        connect(mCheckboxs[i], SIGNAL(stateChanged(int)), this, SLOT(updateGroups()));


    }

    mLayout.setRowStretch(slam->getMap().getMapPointsGroupsManager().getGroupNumber(), 1);

    mCheckboxs[2]->setChecked(true);

    setLayout(&mLayout);

    updateGroups();

}

CML::GroupsWidget::~GroupsWidget() {

    for (int i = 0; i < MAXGROUPSIZE; i++) {
        if (mLabels[i] != nullptr) {
            delete mLabels[i];
            delete mCheckboxs[i];
        }
    }

}

void CML::GroupsWidget::updateGroups() {
    unsigned int groups = 0;

    for (int i = 0; i < MAXGROUPSIZE; i++) {
        if (mCheckboxs[i] != nullptr) {
            if (mCheckboxs[i]->isChecked()) {
                groups |= 1U << i;
            }
        }
    }

    if (groups == 0) {
        for (int i = 0; i < MAXGROUPSIZE; i++) {
            groups |= 1U << i;
        }
    }

    emit onGroupsChanged(groups);
}