//
// Created by tbelos on 17/04/19.
//

#include <stdexcept>

#include "cml/capture/VideoCapture.h"
#include "cml/map/InternalCalibration.h"

#if CML_HAVE_AVCODEC

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

inline std::string av_strerror(int errnum) {
    char buffer[1024];
    av_strerror(errnum, buffer, 1024);
    return buffer;
}

std::string dirnameOf(const std::string& fname)
{
    size_t pos = fname.find_last_of("\\/");
    return (std::string::npos == pos)
           ? ""
           : fname.substr(0, pos);
}

CML::CaptureFFMPEG::CaptureFFMPEG(const std::string &path, unsigned int height, size_t cacheSize) :
    mPath(path), mHeight(height) {


    // Open video file
    int errorCode = avformat_open_input(&mFormatCtx, path.c_str(), nullptr, nullptr);
    if(errorCode != 0) {
        throw std::runtime_error("Can't open '" + path + ". " + av_strerror(errorCode));
    }

    // Retrieve stream information
    if(avformat_find_stream_info(mFormatCtx, nullptr)<0) {
        throw std::runtime_error("Could not find stream information for '" + path + "'");
    }

    av_dump_format(mFormatCtx, 0, path.c_str(), 0);

    // Find the first video stream
    for(unsigned int i = 0; i<mFormatCtx->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = mFormatCtx->streams[i]->codecpar;
        AVCodec *pLocalCodec = (AVCodec*)avcodec_find_decoder(pLocalCodecParameters->codec_id);


        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            mVideoStream = i;
            mCodec = pLocalCodec;
            mCodecParameters = pLocalCodecParameters;
            if (mCodec != nullptr) break;
        }
    }

    if(mVideoStream==-1) {
        throw std::runtime_error("Could not find any compatible video stream for '" + path + "'");
    }


    // Copy context
    mCodecCtx = avcodec_alloc_context3(mCodec);
    if (!mCodecCtx) {
        throw std::runtime_error("Couldn't allocate codec context for '" + path + "'");
    }

    if (avcodec_parameters_to_context(mCodecCtx, mCodecParameters) < 0) {
        fprintf(stderr, "Couldn't copy codec context");
        throw std::runtime_error("Couldn't copy codec context for '" + path + "'");
    }

    // Open codec
    if(avcodec_open2(mCodecCtx, mCodec, nullptr) < 0) {
        throw std::runtime_error("Couldn't open codec for '" + path + "'");
    }

    mFrame = av_frame_alloc();
    if (!mFrame) {
        throw std::runtime_error("Couldn't allocate frame for '" + path + "'");
    }

    //mPacket = av_packet_alloc();
    //if (!mPacket) {
    //    throw std::runtime_error("Couldn't allocate packet for '" + path + "'");
   // }

    CML_LOG_INFO("Video '" + std::string(path) + "' is open !");

    if (mHeight == 0) {
        mWidth = mCodecCtx->width;
        mHeight = mCodecCtx->height;
    } else {
        mWidth = mCodecCtx->width * mHeight / mCodecCtx->height;
    }

    CML_LOG_INFO("The images will be resized to " + std::to_string(mWidth) + "x" + std::to_string(mHeight));

    mVignette = Array2D<float>(mWidth, mHeight, 1.0f);

    mCaptureImageGenerator = new CaptureImageGenerator(mWidth, mHeight);
    try {
        mCameraParameters = parseInternalTumCalibration(path + ".txt", mCaptureImageGenerator->getOutputSize());
    } catch (...) {
        CML::Vector2 originalSize(640,480);
        CML::PinholeUndistorter undistorter(CML::Vector2(1.0, 1.7778), CML::Vector2(0.5, 0.5));
        undistorter = undistorter.scaleAndRecenter(originalSize, CML::Vector2(-0.5, -0.5));
        mCameraParameters = new CML::InternalCalibration(undistorter, originalSize);

        //mCameraParameters = parseInternalTumCalibration(dirnameOf(path) + "/calib.txt", mCaptureImageGenerator->getOutputSize());
    }

    mThread = std::thread(&CaptureFFMPEG::ffmpegRun, this);

}

CML::CaptureFFMPEG::~CaptureFFMPEG() {

    av_frame_free(&mFrame);

    avcodec_close(mCodecCtx);
    avcodec_free_context(&mCodecCtx);

    avformat_close_input(&mFormatCtx);

    delete mCaptureImageGenerator;

    delete mCameraParameters;

}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::CaptureFFMPEG::next() {
    Ptr<CaptureImage, Nullable> result = *mQueue.getPopElement();
    mQueue.notifyPop();
    return result;
}

void CML::CaptureFFMPEG::ffmpegRun() {

    CML_LOG_INFO("Playing video '" + std::string(mPath) + "'");

    AVPacket packet;

    double fps = av_q2d(mFormatCtx->streams[mVideoStream]->r_frame_rate);
    int f = 0;

    while(av_read_frame(mFormatCtx, &packet)>=0) {

        if (packet.stream_index == mVideoStream) {

            int response = avcodec_send_packet(mCodecCtx, &packet);
            if (response < 0) {
                // std::string error = av_err2str(response);
                throw std::runtime_error("Error while sending a packet to the decoder");
            }

            while (response >= 0)
            {

                response = avcodec_receive_frame(mCodecCtx, mFrame);
                if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
                    break;
                } else if (response < 0) {
                    // std::string error = av_err2str(response);
                    throw std::runtime_error("Error while receiving a frame from the decoder");
                }

                if (response >= 0) {

                    // Create the frame
                    unsigned int width = mFrame->width;
                    unsigned int height = mFrame->height;

                    Image image(mWidth, mHeight);
                    uint8_t *const data[8] = {(uint8_t*)image.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
                    const int stride[8] = {(int)(width * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};

                    SwsContext *sws_ctx = sws_getContext(width, height, (AVPixelFormat)mFrame->format, mWidth, mHeight, AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);
                    sws_scale(sws_ctx, (uint8_t const * const *)mFrame->data, mFrame->linesize, 0, height, data, stride);
                    sws_freeContext(sws_ctx);

                    *mQueue.getPushElement() = mCaptureImageGenerator->create()
                            .setImage(image)
                            .setPath(mPath)
                            .setTime((double)f / fps)
                            .setCalibration(mCameraParameters)
                            //.setLut(&mLookupTable)
                            //.setInverseVignette(mVignette)
                            .generate();
                    f = f + 1;
                    mQueue.notifyPush();

                }

            }

        }

        av_packet_unref(&packet);

    }

    CML_LOG_INFO("Video finisehd !");

    *mQueue.getPushElement() = Ptr<CaptureImage, Nullable>();
    mQueue.notifyPush();

}

CML::VideoCapture::VideoCapture(const std::string &path, unsigned int height, size_t cacheSize) {
    mCapturePtr = std::make_shared<CaptureFFMPEG>(path, height, cacheSize);
    mInit = true;
}

void CML::VideoCapture::play() {
    mCapturePtr->play();
}

void CML::VideoCapture::stop() {
    mCapturePtr->stop();
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::VideoCapture::next() {
    return mCapturePtr->next();
}

bool CML::VideoCapture::isInit() {
    return mInit;
}

#endif
