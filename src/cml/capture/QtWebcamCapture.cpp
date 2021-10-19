//
// Created by tbelos on 22/08/19.
//


#include "cml/capture/QtWebcamCapture.h"

#if CML_ENABLE_GUI

#include "cml/utils/Logger.h"
#include "cml/image/Array2D.h"
#include "cml/map/InternalCalibration.h"
#include <unistd.h>

QtWebcamCapture::QtWebcamCapture(size_t poolSize, QObject *parent) : QVideoSink(parent)
{
    mMediaCaptureSession = new QMediaCaptureSession();
    mCamera = new QCamera(QCameraDevice::BackFace);

    mMediaCaptureSession->setCamera(mCamera);
    mMediaCaptureSession->setVideoSink(this);

    mCamera->start();
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

inline int QtWebcamCapture::remaining() {
    return -1;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> QtWebcamCapture::next() {
    QVideoFrame frame;
    while (!frame.isValid() || frame.width() == 0 || frame.height() == 0) {
        frame = videoFrame();
    }
    if (frame.isValid()) {

        frame.map(QVideoFrame::ReadOnly);

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

        return nextFrame;

    } else {
        return nullptr;
    }
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

#endif