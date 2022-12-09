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

class QtWebcamCapture : public QVideoSink, public CML::AbstractRealtimeCapture {


    Q_OBJECT

public:
    explicit QtWebcamCapture(size_t poolSize = 10, QObject *parent = 0);

    ~QtWebcamCapture();

    bool isInit();

    void play();

    void stop();

    int remaining();

    CML::Ptr<CML::CaptureImage, CML::Nullable> next();

    void setExposure(float exposure) final;

    float getMinimumExposure() final;

    float getMaximumExposure() final;

    void setAutoExposure(bool value) final;

    bool isAutoExposure() final;

    inline CML::Ptr<CML::CaptureImageGenerator, CML::Nullable> getGenerator() final {
        return mCaptureImageGenerator;
    }

public slots:
    void hvideoFrameChanged(const QVideoFrame &frame);

private:

    QMediaCaptureSession *mMediaCaptureSession;
    QCamera *mCamera;

    std::mutex mFrameMutex;
    CML::Ptr<CML::CaptureImage, CML::Nullable> mFrame;
    CML::InternalCalibration *mCalibration = nullptr;

    CML::CaptureImageGenerator *mCaptureImageGenerator;

    CML::GrayLookupTable mLookupTable;

    CML::Array2D<float> mVignette;

    CML::Queue<CML::Ptr<CML::CaptureImage, CML::Nullable>, 1> nextFrames;

};

#endif


#endif //CML_ANDROIDCAPTURE_H
