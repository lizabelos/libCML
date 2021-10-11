//
// Created by tbelos on 22/08/19.
//

#include "cml/capture/QtWebcamCapture.h"
#include "cml/utils/Logger.h"
#include "cml/image/Array2D.h"
#include "cml/map/InternalCalibration.h"
#include <unistd.h>

QtWebcamCapture::QtWebcamCapture(size_t poolSize, QObject *parent) : QAbstractVideoSurface(parent)
{
    mCamera = new QCamera(QCamera::BackFace);

    QCameraViewfinderSettings viewfinderSettings;
    viewfinderSettings.setResolution(640, 480);
    viewfinderSettings.setMaximumFrameRate(30);
    mCamera->setViewfinderSettings(viewfinderSettings);

    mCamera->setViewfinder(this);

    mVignette = CML::Array2D<float>(640, 480, 1);
    mCaptureImageGenerator = new CML::CaptureImageGenerator(640, 480);

}

QList<QVideoFrame::PixelFormat> QtWebcamCapture::supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType) const
{
    Q_UNUSED(handleType);
    return QList<QVideoFrame::PixelFormat>() << QVideoFrame::Format_RGB32;
}

bool QtWebcamCapture::present(const QVideoFrame &frame)
{
    if (frame.isValid()) {
        QVideoFrame cloneFrame(frame);
        cloneFrame.map(QAbstractVideoBuffer::ReadOnly);

        CML::Image image(cloneFrame.width(), cloneFrame.height());
        memcpy(image.data(), cloneFrame.bits(), cloneFrame.width() * cloneFrame.height() * 4);

        if (mCalibration == nullptr) {
            mCalibration = new CML::InternalCalibration();
            // *mCalibration = CML::getAndroidCameraParameters(cloneFrame.width(), cloneFrame.height(), "unknown"); // TODO
        }


        // todo : get exposure
        CML::Ptr<CML::CaptureImage, CML::Nullable> nextFrame = mCaptureImageGenerator->create()
                .setImage(image.resize(640, 480))
                .setTime( (float)frame.startTime() / 1e6f)
                .setCalibration(mCalibration)
                .setLut(&mLookupTable)
                .setInverseVignette(mVignette)
                .generate();

        mFrameMutex.lock();
        mFrame = nextFrame;
        mFrameMutex.unlock();

        cloneFrame.unmap();
        return true;
    }
    return false;
}

bool QtWebcamCapture::isInit() {
    return true;
}

void QtWebcamCapture::play() {
    CML::logger.info("Starting camera...");
    mCamera->start();
}

void QtWebcamCapture::stop() {
    CML::logger.info("Stoping camera...");
    mCamera->stop();
}

inline int QtWebcamCapture::remaining() {
    return -1;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> QtWebcamCapture::next() {
    while (mFrame.isNull()) {
        usleep(10);
    }
    mFrameMutex.lock();
    CML::Ptr<CML::CaptureImage, CML::Nullable> frame = mFrame;
    mFrame = nullptr;
    mFrameMutex.unlock();
    return frame;
}

void QtWebcamCapture::setExposure(float exposure) {
    mCamera->exposure()->setManualShutterSpeed(exposure);
}

float QtWebcamCapture::getMinimumExposure() {
    QList<qreal> supportedShutterSpeeds = mCamera->exposure()->supportedShutterSpeeds();
    return *std::min_element(supportedShutterSpeeds.begin(), supportedShutterSpeeds.end());
}

float QtWebcamCapture::getMaximumExposure() {
    QList<qreal> supportedShutterSpeeds = mCamera->exposure()->supportedShutterSpeeds();
    return *std::max_element(supportedShutterSpeeds.begin(), supportedShutterSpeeds.end());
}

void QtWebcamCapture::setAutoExposure(bool value) {
    if (value) {
        mCamera->exposure()->setExposureMode(QCameraExposure::ExposureAuto);
    } else {
        mCamera->exposure()->setExposureMode(QCameraExposure::ExposureManual);
    }
}

bool QtWebcamCapture::isAutoExposure() {
    return mCamera->exposure()->exposureMode() == QCameraExposure::ExposureAuto;
}