//
// Created by tbelos on 22/08/19.
//

#ifndef CML_ANDROIDCAPTURE_H
#define CML_ANDROIDCAPTURE_H

#include "cml/config.h"

#if CML_ENABLE_GUI

#include <QImage>
#include <QList>
#include <mutex>
#include <QtMultimedia/QCamera>
#include <QtMultimedia/QVideoSink>
#include <QtMultimedia/QMediaCaptureSession>

#include "cml/capture/AbstractCapture.h"
#include "cml/image/LookupTable.h"

class QtWebcamCapture : public CML::AbstractRealtimeCapture, public QVideoSink {


public:
    explicit QtWebcamCapture(size_t poolSize = 10, QObject *parent = 0);

    ~QtWebcamCapture();

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
    QMediaCaptureSession *mMediaCaptureSession;
    QCamera *mCamera;

    std::mutex mFrameMutex;
    CML::Ptr<CML::CaptureImage, CML::Nullable> mFrame;
    CML::InternalCalibration *mCalibration = nullptr;

    CML::CaptureImageGenerator *mCaptureImageGenerator;

    CML::GrayLookupTable mLookupTable;

    CML::Array2D<float> mVignette;

};

#endif


#endif //CML_ANDROIDCAPTURE_H
