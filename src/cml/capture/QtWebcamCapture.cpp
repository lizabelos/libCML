//
// Created by tbelos on 22/08/19.
//


#include "cml/capture/QtWebcamCapture.h"

#if CML_ENABLE_GUI

#include <QMediaDevices>

#include "cml/utils/Logger.h"
#include "cml/image/Array2D.h"
#include "cml/map/InternalCalibration.h"
#include <unistd.h>

QtWebcamCapture::QtWebcamCapture(size_t poolSize, QObject *parent) : QVideoSink(parent)
{

    connect(this, &QVideoSink::videoFrameChanged, this, &QtWebcamCapture::hvideoFrameChanged);

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    qDebug() << "Found " << cameras.size() << " cameras";
    for (const QCameraDevice &cameraDevice : cameras) {
        qDebug() << cameraDevice.description();
    }

    qDebug() << "new QMediaCaptureSession";
    mMediaCaptureSession = new QMediaCaptureSession();

    qDebug() << "new QCamera";
    mCamera = new QCamera(cameras[0]);

    int settingsIndex = 0;
    const auto settings = mCamera->cameraDevice().videoFormats();
    for (int i = 0; i < settings.size(); i++) {
        auto setting = settings[i];
        qDebug() << "Format : " << setting.resolution() << " at " << setting.minFrameRate() << "-" << setting.maxFrameRate();
        if (settingsIndex == 0 && setting.resolution().width() == 640 && setting.resolution().height() == 480) {
            settingsIndex = i;
            qDebug() << "Choosing this one !";
        }
    }

    const auto s = settings.at( settingsIndex );

    //QVideoFrame frame(QVideoFrameFormat(s.resolution(), s.pixelFormat()));
    //mSink->setVideoFrame(frame);

    mCamera->setFocusMode( QCamera::FocusModeAuto );
    mCamera->setCameraFormat( s );



    qDebug() << "mMediaCaptureSession->setCamera";
    mMediaCaptureSession->setCamera(mCamera);

    qDebug() << "mMediaCaptureSession->setVideoOutput";
    mMediaCaptureSession->setVideoSink(this);

    qDebug() << "mCamera->start";
    mCamera->start();

    qDebug() << "Camera started !";
}

QtWebcamCapture::~QtWebcamCapture() {
    delete mMediaCaptureSession;
    delete mCamera;
}

bool QtWebcamCapture::isInit() {
    return true;
}

void QtWebcamCapture::play() {
}

void QtWebcamCapture::stop() {

}

int QtWebcamCapture::remaining() {
    return -1;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> QtWebcamCapture::next() {
    CML::Ptr<CML::CaptureImage, CML::Nullable> nextFrame = *nextFrames.getPopElement();
    nextFrames.notifyPop();
    return nextFrame;
}

void QtWebcamCapture::setExposure(float exposure) {
    mCamera->setManualExposureTime(exposure);
}

float QtWebcamCapture::getMinimumExposure() {
    return mCamera->minimumExposureTime();
}

float QtWebcamCapture::getMaximumExposure() {
    return mCamera->maximumExposureTime();
}

void QtWebcamCapture::setAutoExposure(bool value) {
    if (value) {
        mCamera->setExposureMode(QCamera::ExposureMode::ExposureAuto);
    } else {
        mCamera->setExposureMode(QCamera::ExposureMode::ExposureManual);
    }
}

bool QtWebcamCapture::isAutoExposure() {
    return mCamera->exposureMode() == QCamera::ExposureMode::ExposureAuto;
}

void QtWebcamCapture::hvideoFrameChanged(const QVideoFrame &frame) {
    //frame.map(QVideoFrame::ReadOnly);

    if (nextFrames.getCurrentSize() == 1) {
        return;
    }

    if (frame.isValid()) {
        qDebug() << "Valid next frame";

        CML::logger.important("Webcam new frame : " + std::to_string(frame.width()) + "x" + std::to_string(frame.height()));

        QImage qimage = frame.toImage();
        qimage.convertTo(QImage::Format_RGBA8888);
        qimage = qimage.scaledToWidth(640);

        CML::Image image(qimage.width(), qimage.height());
        memcpy(image.data(), qimage.bits(), qimage.width() * qimage.height() * 4);

        if (mCalibration == nullptr) {
            // Todo : this is the parameters for a google pixel 3a
            CML::Vector2 originalSize(qimage.width(), qimage.height());
            CML::PinholeUndistorter undistorter(CML::Vector2(1.0, 1.7778), CML::Vector2(0.5, 0.5));
            undistorter = undistorter.scaleAndRecenter(originalSize, CML::Vector2(-0.5, -0.5));
            mCalibration = new CML::InternalCalibration(undistorter, originalSize);


            mVignette = CML::Array2D<float>(qimage.width(), qimage.height(), 1);
            mCaptureImageGenerator = new CML::CaptureImageGenerator(qimage.width(), qimage.height());

        }

        CML::Ptr<CML::CaptureImage, CML::Nullable> nextFrame = mCaptureImageGenerator->create()
                .setImage(image)
                .setTime( (float)frame.startTime() / 1e6f)
                .setCalibration(mCalibration)
                .setLut(&mLookupTable)
                .setInverseVignette(mVignette)
                .setExposure(mCamera->exposureTime())
                .generate();

        *nextFrames.getPushElement() = nextFrame;
        nextFrames.notifyPush();
    }
}

#endif
