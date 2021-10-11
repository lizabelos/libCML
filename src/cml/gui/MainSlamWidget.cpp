#include "cml/gui/MainSlamWidget.h"

#include <exception>

#if CML_HAVE_AVFORMAT
#include "cml/capture/VideoCapture.h"
#endif

#if CML_HAVE_LIBZIP
#include "cml/capture/TUMCapture.h"
#endif

#include "cml/capture/KittyCapture.h"
#include "cml/utils/Logger.h"
#include "cml/base/AbstractSlam.h"
// #include "cml/gui/capture/QtWebcamCapture.h"


CML::MainSlamWidget::MainSlamWidget(Ptr<AbstractSlam, NonNullable> slam, bool renderMode):
    mSLAM(slam),
    mModelWidget(slam.p(), FOLLOW),
    mFunctionListWidget(slam.p()),
    mGroupsWidget(slam.p()),
    mGarbageCollectorInstance(slam->getMap().getGarbageCollector().newInstance())
{

    setObjectName("MainSlamWidget");

    setWindowTitle("CML by Thomas Belos");

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


    mButtonPlay.setText("Play");
    mButtonPlay.setCheckable(true);
    mButtonPlay.setChecked(!slam->isPaused());
    mCommandsLayout.addWidget(&mButtonPlay);
    mButtonNext.setText("Next");
    mCommandsLayout.addWidget(&mButtonNext);
    mFilterVariance.setMaximum(std::numeric_limits<double>::max());
    mFilterVariance.setMinimum(0);
    mFilterVariance.setDecimals(10);
    mFilterVariance.setValue(1000000000);
    mCommandsLayout.addWidget(&mFilterVariance);
    mCommandsLayout.addStretch();

    if (!renderMode) {
        mFunctionListWidget.setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
    } else {
        mModelWidget.setFixedSize(3840, 2160);
    }
    mModelWidget.setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    mGroupsWidget.setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));

   // if (isBigScreen) {
    //    for (int i = 0; i < 7; i++) {
    //        mCameraViewerWidgets.emplace_back(new CameraViewerWidget(slam.p(), i - 1));
    //        mCameraViewerWidgets[i]->setMinimumHeight(200.0f / 96.0f * QDesktopWidget().logicalDpiX());
    //        mCameraViewerLayout.addWidget(mCameraViewerWidgets[i]);
    //    }
    //} else {
        mCameraViewerWidgets.emplace_back(new CameraViewerWidget(slam.p(), -1));
    mCameraViewerWidgets[0]->setMinimumHeight(200.0f / 96.0f * QDesktopWidget().logicalDpiX());
    mCameraViewerLayout.addWidget(mCameraViewerWidgets[0]);
    //}

    mInformationsLayout.addWidget(&mRamUsageLabel);
    mInformationsLayout.addWidget(&mGarbageLabel);
    mInformationsLayout.addWidget(&mNumFrameLabel);
    mInformationsLayout.addWidget(&mNumKeyframeLabel);
    mInformationsLayout.addWidget(&mNumPointLabel);
    mInformationsLayout.addWidget(&mNumSegmentLabel);
    mInformationsLayout.addWidget(&mCameraPoseLabel);
    mInformationsLayout.addStretch();

    if (isBigScreen) {

        mLayout.addLayout(&mCommandsLayout, 0, 0, 1, 3);
        mLayout.addWidget(&mFunctionListWidget, 1, 0);
        mLayout.addWidget(&mModelWidget, 1, 1);
        mLayout.addWidget(&mGroupsWidget, 1, 2);
        mLayout.addLayout(&mCameraViewerLayout, 2, 0, 1, 3);
        mLayout.addLayout(&mInformationsLayout, 3, 0, 1, 3);

        setLayout(&mLayout);

        resize(QDesktopWidget().availableGeometry(this).size() * 0.95);

    } else {

        mTabWidget.addTab(&mFunctionListWidget, "Functions");
        mTabWidget.addTab(&mModelWidget, "Model");
        mTabWidget.addTab(&mGroupsWidget, "Groups");
        mTabWidget.addTab(mCameraViewerWidgets[0], "Camera");
        mTabWidget.setCurrentIndex(1);

        mLayout.addLayout(&mCommandsLayout, 0, 0);
        mLayout.addWidget(&mTabWidget, 1, 0);
        setLayout(&mLayout);

    }

    connect(&mFilterVariance, SIGNAL(valueChanged(double)), this, SLOT(onVarianceFilterChange(double)));
    connect(&mGroupsWidget, SIGNAL(onGroupsChanged(unsigned int)), &mModelWidget, SLOT(updateGroupsFilter(unsigned int)));
    connect(&mButtonPlay, SIGNAL(clicked()), this, SLOT(play()));

    mModelWidget.setVarianceFilter(mFilterVariance.value());
    mGroupsWidget.updateGroups();

    mTimer.start(1000 / 5, this);

}

CML::MainSlamWidget::~MainSlamWidget() {
    for (auto widget : mCameraViewerWidgets) {
        delete widget;
    }
}

void CML::MainSlamWidget::play() {
    mSLAM->setPaused(!mSLAM->isPaused());
    mButtonPlay.setChecked(!mSLAM->isPaused());
}

void CML::MainSlamWidget::next() {
    mSLAM->next();
}

int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}


void CML::MainSlamWidget::timerEvent(QTimerEvent *e) {

    float memoryUsage = CML::memoryUsage();
    mRamUsageLabel.setText(QString::fromStdString(std::to_string(memoryUsage) + " MB used"));

    if (mSLAM == nullptr) {
        return;
    }

    mGarbageLabel.setText(QString::fromStdString(std::to_string(mSLAM->getMap().getGarbageCollector().numGarbages()) + " garbages"));
    mNumFrameLabel.setText(QString::fromStdString(std::to_string(mSLAM->getMap().getFramesNumber()) + " frames"));
    mNumKeyframeLabel.setText(QString::fromStdString(std::to_string(mSLAM->getMap().getGroupFramesNumber(mSLAM->getMap().KEYFRAME)) + " keyframes"));
    mNumPointLabel.setText(QString::fromStdString(std::to_string(mSLAM->getMap().getMapPointsNumber()) + " points"));

    if (mSLAM->getMap().getFramesNumber() > 0) {
        CML::PFrame frame = mSLAM->getMap().getLastFrame();

        std::stringstream ss;
        ss << frame->getCamera() << "; C : " << frame->getCalibration().getParameters(0).transpose() << "; E : [" << frame->getExposure().getParameters().transpose() << "]; ";

        mCameraPoseLabel.setText(QString::fromStdString(ss.str()));

    }

}


void CML::MainSlamWidget::paintEvent(QPaintEvent *e) {
    if (mSLAM != nullptr) {
        mSLAM->getMap().getGarbageCollector().collect(mGarbageCollectorInstance);
    }
}

void CML::MainSlamWidget::onVarianceFilterChange(double value) {
    mModelWidget.setVarianceFilter(value);
}
