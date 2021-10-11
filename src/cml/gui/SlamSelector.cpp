//
// Created by thomas on 06/12/2020.
//

#include "cml/gui/SlamSelector.h"

#include <QDir>
#include <QDirIterator>
#include <QFileDialog>

CML::SlamSelector::SlamSelector() {

    setObjectName("SlamSelector");

    mLabelInstructions.setText("Please select a SLAM : ");
    mLayout.addWidget(&mLabelInstructions, 0, 0);

    mLabelError.setText("");
    mLayout.addWidget(&mLabelError, 1, 0);


    int i = 3;

    char buff[FILENAME_MAX];

#ifdef WIN32
    GetCurrentDirectory(FILENAME_MAX, buff);
#else
    getcwd( buff, FILENAME_MAX );
#endif
    std::string current_working_dir(buff);
    std::cout << current_working_dir << std::endl;

    {
        QDirIterator it("modules", QStringList() << "*." DL_EXT, QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            std::string next = it.next().toStdString();
            mButtonsSLAM.emplace_back(new QPushButton(QString::fromStdString(next)));
            connect(mButtonsSLAM[mButtonsSLAM.size() - 1], &QPushButton::pressed, this, [this, next]{ changeSlam(next); });
            mLayout.addWidget(mButtonsSLAM[mButtonsSLAM.size() - 1], i, 0);
            i++;
        }
    }

    {
        QDirIterator it("../modules", QStringList() << "*." DL_EXT, QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            std::string next = it.next().toStdString();
            mButtonsSLAM.emplace_back(new QPushButton(QString::fromStdString(next)));
            connect(mButtonsSLAM[mButtonsSLAM.size() - 1], &QPushButton::pressed, this, [this, next]{ changeSlam(next); });
            mLayout.addWidget(mButtonsSLAM[mButtonsSLAM.size() - 1], i, 0);
            i++;
        }
    }

    {
        QDirIterator it("../../modules", QStringList() << "*." DL_EXT, QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            std::string next = it.next().toStdString();
            mButtonsSLAM.emplace_back(new QPushButton(QString::fromStdString(next)));
            connect(mButtonsSLAM[mButtonsSLAM.size() - 1], &QPushButton::pressed, this, [this, next]{ changeSlam(next); });
            mLayout.addWidget(mButtonsSLAM[mButtonsSLAM.size() - 1], i, 0);
            i++;
        }
    }

    {
        mButtonCustomSLAM.setText("Custom path...");
        connect(&mButtonCustomSLAM, SIGNAL(pressed()), this, SLOT(changeSlamDialog()));
        mLayout.addWidget(&mButtonCustomSLAM, i, 0);
        i++;
    }

    setLayout(&mLayout);

}

CML::SlamSelector::~SlamSelector() {
    for (auto button : mButtonsSLAM) {
        delete button;
    }
}

void CML::SlamSelector::changeSlam(std::string soPath) {
    CML::MakeFunction make = CML::loadExt(soPath);
    mSLAM = make();
}

void CML::SlamSelector::changeSlamDialog() {
    QString extPath = QFileDialog::getOpenFileName(this, "Open Slam module");

    try {
        changeSlam(extPath.toStdString());
    } catch (const std::exception &e) {
        mLabelError.setText(QString::fromStdString(e.what()));
        CML::logger.fatal(e.what());
    }
}