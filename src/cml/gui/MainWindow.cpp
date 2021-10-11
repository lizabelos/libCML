//
// Created by thomas on 07/12/2020.
//

#include "cml/gui/MainWindow.h"

CML::MainWindow::MainWindow() {

    setObjectName("MainWindow");

    bool isBigScreen;
    if (QDesktopWidget().availableGeometry(this).width() > 1200) {
        if (QDesktopWidget().logicalDpiX() < 300) {
            isBigScreen = true;
        } else {
            isBigScreen = false;
        }
    } else {
        isBigScreen = false;
    }

    mLaunchButton.setText("Launch");

    if (isBigScreen) {

        mMainLayout.addWidget(&mCaptureSelector, 0, 0);
        mMainLayout.addWidget(&mSlamSelector, 0, 1);
        mMainLayout.addWidget(&mLaunchButton, 1, 0, 1, 2);

    } else {

        mMainLayout.addWidget(&mCaptureSelector, 0, 0);
        mMainLayout.addWidget(&mSlamSelector, 1, 0);
        mMainLayout.addWidget(&mLaunchButton, 2, 0);

    }

    connect(&mLaunchButton, SIGNAL(pressed()), this, SLOT(launch()));

    setLayout(&mMainLayout);

}

void CML::MainWindow::launch() {

    auto capture = mCaptureSelector.getCapture();
    auto slam = mSlamSelector.getSLAM();

    if (capture.isNotNull() && slam.isNotNull()) {
        slam->start(capture);
        mMainSlamWidget = new MainSlamWidget(slam);
        mMainSlamWidget->showMaximized();
        hide();
    }

}