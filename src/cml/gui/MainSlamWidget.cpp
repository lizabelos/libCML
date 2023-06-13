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
#include "cml/capture/QtWebcamCapture.h"

#include <QSettings>
#include <QScreen>
#include <QStyleFactory>
#include <QDir>

CML::MainSlamWidget::MainSlamWidget(Ptr<AbstractSlam, NonNullable> slam, bool renderMode):
    mSLAM(slam),
    mModelWidget(slam.p(), FOLLOW),
    mCameraViewerWidgets(slam.p(), -1),
    #ifndef ANDROID
    mFunctionListWidget(slam.p()),
    mGroupsWidget(slam.p()),
    #endif
    mGarbageCollectorInstance(slam->getMap().getGarbageCollector().newInstance())
{

    setObjectName("MainSlamWidget");

    setWindowTitle("libCML GUI");

    CML_LOG_IMPORTANT("Available geometry width : " + std::to_string(screen()->availableGeometry().width()));
    CML_LOG_IMPORTANT("DPI X : " + std::to_string(logicalDpiX()));

    bool isBigScreen;
    /*if (screen()->availableGeometry().width() > 1200) {
        if (logicalDpiX() < 300) {
            isBigScreen = true;
        } else {
            isBigScreen = false;
        }
    } else {
        isBigScreen = false;
    }*/
#if CML_IS_ANDROID
    isBigScreen = false;
#else
    isBigScreen = true;
#endif

       if(!QFileInfo::exists("resources/ORBvoc.zip")) {
           if (!QDir().exists("resources")) {
               if (!QDir().mkdir("resources")) {
                   qDebug() << "Can't create directory resources";
               }
           }
           if (!QFile::copy(":/resources/ORBvoc.zip", "resources/ORBvoc.zip")) {
               qDebug() << "Can't copy ORBvoc.zip";
               abort();
           }
       }

  //  mButtonPlay.setText("Downloading data...");
 //   if(QFileInfo::exists("ORBvoc.zip")) {
      //  qDebug() << "Found ORBvoc.zip";
        mButtonPlay.setText("Play");
        mButtonPlay.setCheckable(true);
        mButtonPlay.setChecked(!slam->isPaused());
   /* } else {
        qDebug() << "Downloading ORBvoc.zip...";
        mButtonPlay.setEnabled(false);
        connect(
                &m_WebCtrl, SIGNAL (finished(QNetworkReply*)),
                this, SLOT (fileDownloaded(QNetworkReply*))
        );

        QNetworkRequest request(QUrl("https://github.com/belosthomas/libCML/raw/main/resources/ORBvoc.zip"));
        m_WebCtrl.get(request);
    }*/

    mCommandsLayout.addWidget(&mButtonPlay);
    mButtonNext.setText("Next");
    mCommandsLayout.addWidget(&mButtonNext);
    mFilterVariance.setMaximum(std::numeric_limits<double>::max());
    mFilterVariance.setMinimum(0);
    mFilterVariance.setDecimals(10);
    mFilterVariance.setValue(1000000000);
    mCommandsLayout.addWidget(&mFilterVariance);
    mCommandsLayout.addStretch();

#ifndef ANDROID
    if (!renderMode) {
        mFunctionListWidget.setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
    } else {
        mModelWidget.setFixedSize(1280, 720);
    }
    mGroupsWidget.setSizePolicy(QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
#endif
    mModelWidget.setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

    mInformationsLayout.addWidget(&mRamUsageLabel);
    mInformationsLayout.addWidget(&mGarbageLabel);
    mInformationsLayout.addWidget(&mNumFrameLabel);
    mInformationsLayout.addWidget(&mNumKeyframeLabel);
    mInformationsLayout.addWidget(&mNumPointLabel);
    mInformationsLayout.addWidget(&mNumSegmentLabel);
    mInformationsLayout.addWidget(&mCameraPoseLabel);
    mInformationsLayout.addStretch();

    if (isBigScreen) {
        mCameraViewerWidgets.setMinimumHeight(200.0f / 96.0f * logicalDpiX());

        mLayout.addLayout(&mCommandsLayout, 0, 0, 1, 3);
#ifndef ANDROID
        mLayout.addWidget(&mFunctionListWidget, 1, 0);
#endif
        mLayout.addWidget(&mModelWidget, 1, 1, 2, 1);
#ifndef ANDROID
        mLayout.addWidget(&mGroupsWidget, 1, 2);
#endif
        mLayout.addWidget(&mCameraViewerWidgets, 2, 0, 1, 1);
        mLayout.addLayout(&mInformationsLayout, 3, 0, 1, 3);

        setLayout(&mLayout);

        resize(screen()->availableGeometry().size() * 0.95);

    } else {
        mCameraViewerWidgets.setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
        
        mLayout.addLayout(&mCommandsLayout, 0, 0, 1, 2);
        mLayout.addWidget(&mModelWidget, 1, 0);
        mLayout.addWidget(&mCameraViewerWidgets, 1, 1);
        setLayout(&mLayout);

    }

    connect(&mFilterVariance, SIGNAL(valueChanged(double)), this, SLOT(onVarianceFilterChange(double)));
#ifndef ANDROID
    connect(&mGroupsWidget, SIGNAL(onGroupsChanged(unsigned int)), &mModelWidget, SLOT(updateGroupsFilter(unsigned int)));
#endif
    connect(&mButtonPlay, SIGNAL(clicked()), this, SLOT(play()));

    mModelWidget.setVarianceFilter(mFilterVariance.value());
#ifndef ANDROID
    mGroupsWidget.updateGroups();
#endif

    mTimer.start(1000 / 5, this);

}

CML::MainSlamWidget::~MainSlamWidget() {

}

void CML::MainSlamWidget::play() {

    if (mSLAM->isStopped()) {
        Ptr<AbstractCapture, Nullable> capture = new QtWebcamCapture();
        capture->play();
        mSLAM->start(capture);
    } else {
        mSLAM->setPaused(!mSLAM->isPaused());
    }
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

    float memoryUsage = CML::OS::memoryUsage();
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

void CML::MainSlamWidget::fileDownloaded(QNetworkReply *pReply) {
    qDebug() << "Downloaded ORBvoc.zip";

    QByteArray data = pReply->readAll();
    QFile file("ORBvoc.zip");
    file.open(QIODevice::WriteOnly);
    file.write(data);
    file.close();

    mButtonPlay.setText("Play");
    mButtonPlay.setCheckable(true);
    mButtonPlay.setChecked(!mSLAM->isPaused());
    mButtonPlay.setEnabled(true);
}
