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
    CML_LOG_INFO((std::string)"GL Version : " + std::string(reinterpret_cast<const char*>(glVersion)));

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

    float h, w;
    float remainingH, remainingW;
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

        w = (float) captureFrame->getWidth(0) * height() / (float) captureFrame->getHeight(0);
        w = w * 2.0 / width();
        remainingW = 2.0f - w;

        if (remainingH < 0) {
            remainingH = 0;
            h = 2;
        }

        if (remainingW < 0) {
            remainingW = 0;
            w = 2;
        }

        if (captureFrame->haveColorImage()) {
            mDrawBoard->texture(captureFrame->getColorImage(0, false), -1 + (remainingW / 2), -1 + (remainingH / 2), w, h);
            mDrawBoard->set2DArea(Eigen::Vector2f(-1 + (remainingW / 2), -1 + (remainingH / 2)), Eigen::Vector2f(1 - (remainingW / 2), 1 - (remainingH / 2)));
        } else {
            auto img = captureFrame->getGrayImage(0, false).cast<ColorRGBA>();
            for (int y = 0; y < img.getHeight(); y++) {
                for (int x = 0; x < img.getWidth(); x++) {
                    if (!captureFrame->getDerivativeImage(0).get(x,y).allFinite()) {
                        img(x,y)=ColorRGBA(255,0,0,255);
                    }
                }
            }
            mDrawBoard->texture(img, -1 + (remainingW / 2), -1 + (remainingH / 2), w, h);
            mDrawBoard->set2DArea(Eigen::Vector2f(-1 + (remainingW / 2), -1 + (remainingH / 2)), Eigen::Vector2f(1 - (remainingW / 2), 1 - (remainingH / 2)));

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
        mDrawBoard->set2DArea(Eigen::Vector2f(-1, -1 + (remainingH / 2)), Eigen::Vector2f(1, 1 - (remainingH / 2)));
    }


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
            CML_LOG_ERROR(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::ErrorType:
            CML_LOG_ERROR(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::DeprecatedBehaviorType:
            CML_LOG_WARN(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::UndefinedBehaviorType:
            CML_LOG_WARN(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PortabilityType:
            CML_LOG_WARN(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PerformanceType:
            //CML_LOG_WARN(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::OtherType:
            //CML_LOG_INFO(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::MarkerType:
            //CML_LOG_INFO(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPushType:
            //CML_LOG_INFO(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPopType:
            //CML_LOG_INFO(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::AnyType:
            //CML_LOG_INFO(debugMessage.message().toStdString());
            break;
    }

}