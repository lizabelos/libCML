//
// Created by tbelos on 22/08/19.
//

#ifndef CML_ANDROIDCAPTURE_H
#define CML_ANDROIDCAPTURE_H

#include <QImage>
#include <QList>
#include <mutex>
#include <QtMultimedia/QCamera>
#include <QtMultimedia/QAbstractVideoSurface>
#include <QtMultimedia/QCameraExposure>

#include "cml/config.h"
#include "cml/capture/AbstractCapture.h"
#include "cml/image/LookupTable.h"

class QtWebcamCapture : public CML::AbstractRealtimeCapture, public QAbstractVideoSurface {


public:
    explicit QtWebcamCapture(size_t poolSize = 10, QObject *parent = 0);

    QList<QVideoFrame::PixelFormat> supportedPixelFormats(QAbstractVideoBuffer::HandleType handleType) const;

    bool present(const QVideoFrame &frame);

    bool isInit();

    void play();

    void stop();

    inline int remaining();

    CML::Ptr<CML::CaptureImage, CML::Nullable> next();

    void setExposure(float exposure) final;

    float getMinimumExposure() final;

    float getMaximumExposure() final;

    void setAutoExposure(bool value) final;

    bool isAutoExposure() final;

private:
    QCamera *mCamera;

    std::mutex mFrameMutex;
    CML::Ptr<CML::CaptureImage, CML::Nullable> mFrame;
    CML::InternalCalibration *mCalibration = nullptr;

    CML::CaptureImageGenerator *mCaptureImageGenerator;

    CML::GrayLookupTable mLookupTable;

    CML::Array2D<float> mVignette;

};


#endif //CML_ANDROIDCAPTURE_H
