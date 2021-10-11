//
// Created by tbelos on 17/04/19.
//

#ifndef CML_VIDEOCAPTURE_H
#define CML_VIDEOCAPTURE_H

#include "cml/config.h"

#if CML_HAVE_AVCODEC

#include <string>
#include <memory>
#include <thread>

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
}

#include "cml/capture/AbstractCapture.h"
#include "cml/capture/CaptureImage.h"
#include "cml/image/LookupTable.h"

namespace CML {

    class CaptureFFMPEG : public AbstractMultithreadFiniteCapture {

    public:
        CaptureFFMPEG(const std::string &path, unsigned int height, size_t cacheSize);
        ~CaptureFFMPEG();

        Ptr<CaptureImage, Nullable> multithreadNext() override;

        void ffmpegRun();

    private:
        std::thread mThread;

        Queue<Ptr<CaptureImage, Nullable>, 2> mQueue;

        AVFormatContext *mFormatCtx = nullptr;
        int mVideoStream = -1;
        AVCodecContext *mCodecCtx;
        AVFrame *mFrame = nullptr;
        AVCodec *mCodec = nullptr;
        AVCodecParameters *mCodecParameters = nullptr;

        InternalCalibration *mCameraParameters;

        std::string mPath;
        unsigned int mWidth;
        unsigned int mHeight;

        CaptureImageGenerator *mCaptureImageGenerator;

        GrayLookupTable mLookupTable;

        Array2D<float> mVignette;

    };

    class VideoCapture : public AbstractCapture {

    public:
        VideoCapture() = default;
        VideoCapture(const std::string &path, unsigned int height = 0, size_t cacheSize = 3);

        bool isInit();

        void play() final;
        void stop() final;

        Ptr<CaptureImage, Nullable> next() final;

    private:
        bool mInit = false;
        std::shared_ptr<CaptureFFMPEG> mCapturePtr;

    };

}

#endif

#endif //CML_VIDEOCAPTURE_H
