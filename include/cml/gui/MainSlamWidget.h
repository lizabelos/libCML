#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "cml/config.h"


#include <QMainWindow>
#include <QGridLayout>
#include <QWidget>
#include <QToolBar>
#include <QMenuBar>
#include <QFileDialog>
#include <QPushButton>
#include <QDockWidget>
#include <QBasicTimer>
#include <QStatusBar>
#include <QErrorMessage>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QSpacerItem>
#include <QTabWidget>
#include "cml/gui/widgets/ModelWidget.h"
#include "cml/gui/widgets/FunctionListWidget.h"
#include "cml/gui/widgets/LoggerWidget.h"
#include "cml/gui/widgets/CameraViewerWidget.h"
#include "cml/base/AbstractSlam.h"
#include "cml/gui/widgets/GroupsWidget.h"

namespace CML {

    class MainSlamWidget : public QFrame {
    Q_OBJECT

    public:
        MainSlamWidget(Ptr<AbstractSlam, NonNullable> slam, bool renderMode = false);

        ~MainSlamWidget();

        inline void saveImagesTo(std::string path) {
            mFilterVariance.setValue(0.0001);
            onVarianceFilterChange(0.0001);
            return mModelWidget.saveImagesTo(path);
        }

    public slots:

        void onVarianceFilterChange(double value);

    private slots:

        void play();

        void next();

    protected:
        void timerEvent(QTimerEvent *e) override;

        void paintEvent(QPaintEvent *e) override;

    private:
        QBasicTimer mTimer;

        QGridLayout mLayout;

        QTabWidget mTabWidget;


        Ptr<AbstractSlam, NonNullable> mSLAM;

        QHBoxLayout mCommandsLayout;
        QHBoxLayout mInformationsLayout;


        ModelWidget mModelWidget;

        QHBoxLayout mCameraViewerLayout;
        List<CameraViewerWidget*> mCameraViewerWidgets;
        // CML::ModelWidget *mTopCameraWidget;

        FunctionListWidget mFunctionListWidget;
        GroupsWidget mGroupsWidget;


        QPushButton mButtonPlay;
        QPushButton mButtonNext;
        QDoubleSpinBox mFilterVariance;

        QLabel mRamUsageLabel;
        QLabel mNumFrameLabel;
        QLabel mNumKeyframeLabel;
        QLabel mNumPointLabel;
        QLabel mGarbageLabel;
        QLabel mNumSegmentLabel;
        QLabel mCameraPoseLabel;


        GarbageCollectorInstance mGarbageCollectorInstance;

    };

}

#endif // MAINWINDOW_H
