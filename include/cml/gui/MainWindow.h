//
// Created by thomas on 07/12/2020.
//

#ifndef CML_MAINWINDOW_H
#define CML_MAINWINDOW_H

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>

#include <cml/gui/CaptureSelector.h>
#include <cml/gui/SlamSelector.h>
#include <cml/gui/MainSlamWidget.h>

namespace CML {

    class MainWindow : public QFrame {

        Q_OBJECT

    public:
        MainWindow();

    public slots:
        void launch();

    private:
        QGridLayout mMainLayout;
        CaptureSelector mCaptureSelector;
        SlamSelector mSlamSelector;
        QPushButton mLaunchButton;

        MainSlamWidget *mMainSlamWidget;

    };

}


#endif //CML_MAINWINDOW_H
