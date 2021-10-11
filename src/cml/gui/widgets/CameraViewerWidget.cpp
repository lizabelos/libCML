//
// Created by tbelos on 22/08/19.
//

#include "cml/gui/widgets/CameraViewerWidget.h"
#include "cml/map/InternalCalibration.h"
#include "cml/maths/Utils.h"


CML::CameraViewerWidget::CameraViewerWidget(AbstractSlam *slam, int keyFrame, QWidget *parent) : QOpenGLWidget(parent), mSLAM(slam), mKeyFrame(keyFrame) {
    
}

CML::CameraViewerWidget::~CameraViewerWidget() {

}

void CML::CameraViewerWidget::initializeGL() {
    initializeOpenGLFunctions();

    const GLubyte *glVersion = glGetString(GL_VERSION);
    logger.info((std::string)"GL Version : " + std::string(reinterpret_cast<const char*>(glVersion)));

    mLogger = new QOpenGLDebugLogger(this);
    mLogger->initialize();
    connect(mLogger, &QOpenGLDebugLogger::messageLogged, this, &CameraViewerWidget::onMessageLogged);
    mLogger->startLogging();

    mDrawBoard = new QtDrawBoard(this);

    QOpenGLWidget::initializeGL();

    if (mKeyFrame < 0) {
        mTimer.start(1000 / 20, this);
    } else {
        mTimer.start(1000 / 5, this);
    }
}

void CML::CameraViewerWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    QOpenGLWidget::resizeGL(w, h);
}

void CML::CameraViewerWidget::paintGL() {
    QPainter painter;
    painter.begin(this);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    OptPFrame currentFrame;

    float h;
    float remainingH;
    int imageWidth, imageHeight;

    if (mKeyFrame < 0) {

        auto captureFrame = mSLAM->getLastCaptureFrame();
        if (captureFrame.isNull()) {
            return;
        }


        imageWidth = captureFrame->getWidth(0);
        imageHeight = captureFrame->getHeight(0);
        h = (float) captureFrame->getHeight(0) * width() / (float) captureFrame->getWidth(0);
        h = h * 2.0 / height();
        remainingH = 2.0f - h;

        if (captureFrame->haveColorImage()) {
            mDrawBoard->texture(captureFrame->getColorImage(0), -1, -1 + (remainingH / 2), 2, h);
        } else {
            mDrawBoard->texture(captureFrame->getGrayImage(0).autoadjustAndClip().cast<ColorRGBA>(), -1, -1 + (remainingH / 2), 2, h);

        }

    } else {
        auto keyFrames = mSLAM->getMap().getGroupFrames(mSLAM->getMap().KEYFRAME);
        std::vector<PFrame> frames(keyFrames.begin(), keyFrames.end());
        if (mKeyFrame >= (int)frames.size()) {
            mDrawBoard->finish();
            return;
        }
        currentFrame = frames[mKeyFrame];

        imageWidth = currentFrame->getWidth(0);
        imageHeight = currentFrame->getHeight(0);
        h = (float) currentFrame->getHeight(0) * width() / (float) currentFrame->getWidth(0);
        h = h * 2.0 / height();
        remainingH = 2.0f - h;

        mDrawBoard->texture(frames[mKeyFrame], -1, -1 + (remainingH / 2), 2, h);

    }

    mDrawBoard->set2DArea(Eigen::Vector2f(-1, -1 + (remainingH / 2)), Eigen::Vector2f(1, 1 - (remainingH / 2)));
    mDrawBoard->set2DAxis(Eigen::Vector2f(0, 0), Eigen::Vector2f(imageWidth, imageHeight));


   /*
        mDrawBoard->pointSize(1);

        for (auto featurePoint : currentFrame->getFeaturePoints()) {

            mDrawBoard->color(1, 0, 0);
            mDrawBoard->point(featurePoint.cast<float>());

        }
     */


   if (currentFrame == nullptr && mSLAM->getMap().getFramesNumber() > 0) {
       currentFrame =  mSLAM->getMap().getLastFrame();
   }

   if (!currentFrame.isNull()) {
       mSLAM->viewOnCapture(*mDrawBoard, currentFrame);
   }


    mDrawBoard->finish();

}

void CML::CameraViewerWidget::timerEvent(QTimerEvent *e) {
    update();
}

void CML::CameraViewerWidget::onMessageLogged(const QOpenGLDebugMessage &debugMessage) {

    switch (debugMessage.type()) {

        case QOpenGLDebugMessage::InvalidType:
            logger.error(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::ErrorType:
            logger.error(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::DeprecatedBehaviorType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::UndefinedBehaviorType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PortabilityType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PerformanceType:
            //logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::OtherType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::MarkerType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPushType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPopType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::AnyType:
            //logger.info(debugMessage.message().toStdString());
            break;
    }

}