#include "cml/image/Array2D.h"

#if CML_HAVE_AVFORMAT
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}
#define DEFAULT_RESIZE_ALGORITHM SWS_FAST_BILINEAR // Bicubic seems to be better than bilinear. Todo : try other algorithm
#endif

#define CML_IMAGE_HORIZONTALFLIP false

namespace CML {

    Atomic<size_t> __array2DCounter = 0;

#if CML_HAVE_AVFORMAT
    class FFMPEGContext {

    public:
        FFMPEGContext() {

        }

        ~FFMPEGContext() {
            if (mFrame != nullptr) {
                av_frame_free(&mFrame);
            }
            if (mCodecCtx != nullptr) {
                avcodec_close(mCodecCtx);
                avcodec_free_context(&mCodecCtx);
            }
            if (mFormatCtx != nullptr) {
                avformat_close_input(&mFormatCtx);
            }
        }

        AVFormatContext *mFormatCtx = nullptr; // ok
        int mVideoStream = -1;
        AVCodecContext *mCodecCtx = nullptr; // ok
        AVFrame *mFrame = nullptr; // ok
        AVCodec *mCodec = nullptr;
        AVCodecParameters *mCodecParameters = nullptr;
    };
#endif
}

#if CML_HAVE_AVFORMAT
inline std::string av_strerror(int errnum) {
    char buffer[1024];
    av_strerror(errnum, buffer, 1024);
    return buffer;
}
#endif

CML::Image CML::loadImage(std::string path) {

#if CML_HAVE_AVFORMAT
    logger.debug("Load image " + path);

    FFMPEGContext ctx;

    // Open video file
    int errorCode = avformat_open_input(&ctx.mFormatCtx, path.c_str(), nullptr, nullptr);
    if(errorCode != 0) {
        throw std::runtime_error("Can't open '" + path + ". " + av_strerror(errorCode));
    }

    // Retrieve stream information
    if(avformat_find_stream_info(ctx.mFormatCtx, nullptr)<0) {
        throw std::runtime_error("Could not find stream information for '" + path + "'");
    }

    //av_dump_format(mFormatCtx, 0, path.c_str(), 0);

    // Find the first video stream
    for(unsigned int i = 0; i<ctx.mFormatCtx->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = ctx.mFormatCtx->streams[i]->codecpar;
        AVCodec *pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);


        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            ctx.mVideoStream = i;
            ctx.mCodec = pLocalCodec;
            ctx.mCodecParameters = pLocalCodecParameters;
            if (ctx.mCodec != nullptr) break;
        }
    }

    if(ctx.mVideoStream==-1) {
        throw std::runtime_error("Could not find any compatible video stream for '" + path + "'");
    }


    // Copy context
    ctx.mCodecCtx = avcodec_alloc_context3(ctx.mCodec);
    if (!ctx.mCodecCtx) {
        throw std::runtime_error("Couldn't allocate codec context for '" + path + "'");
    }

    if (avcodec_parameters_to_context(ctx.mCodecCtx, ctx.mCodecParameters) < 0) {
        fprintf(stderr, "Couldn't copy codec context");
        throw std::runtime_error("Couldn't copy codec context for '" + path + "'");
    }

    // Open codec
    if(avcodec_open2(ctx.mCodecCtx, ctx.mCodec, nullptr) < 0) {
        throw std::runtime_error("Couldn't open codec for '" + path + "'");
    }

    ctx.mFrame = av_frame_alloc();
    if (!ctx.mFrame) {
        throw std::runtime_error("Couldn't allocate frame for '" + path + "'");
    }

    //mPacket = av_packet_alloc();
    //if (!mPacket) {
    //    throw std::runtime_error("Couldn't allocate packet for '" + path + "'");
    // }

    AVPacket packet;

    while(av_read_frame(ctx.mFormatCtx, &packet)>=0) {

        if (packet.stream_index == ctx.mVideoStream) {

            int response = avcodec_send_packet(ctx.mCodecCtx, &packet);
            if (response < 0) {
                throw std::runtime_error("Error while sending a packet to the decoder");
            }

            while (response >= 0)
            {

                response = avcodec_receive_frame(ctx.mCodecCtx, ctx.mFrame);
                if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
                    break;
                } else if (response < 0) {
                    // std::string error = av_err2str(response);
                    throw std::runtime_error("Error while receiving a frame from the decoder");
                }

                if (response >= 0) {

                    // Create the frame
                    unsigned int width = ctx.mFrame->width;
                    unsigned int height = ctx.mFrame->height;

                    Image image(width, height);

                    uint8_t *const data[8] = {(uint8_t*)image.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
                    const int stride[8] = {(int)(width * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};

                    SwsContext *sws_ctx = sws_getContext(width, height, (AVPixelFormat)ctx.mFrame->format, width, height, AV_PIX_FMT_RGBA, DEFAULT_RESIZE_ALGORITHM, NULL, NULL, NULL);
                    sws_scale(sws_ctx, (uint8_t const * const *)ctx.mFrame->data, ctx.mFrame->linesize, 0, height, data, stride);
                    sws_freeContext(sws_ctx);

                    av_packet_unref(&packet);

                    if (CML_IMAGE_HORIZONTALFLIP) {
                        return image.horizontalFlip();
                    } else {
                        return image;
                    }

                }

            }

        }

        av_packet_unref(&packet);

    }

    throw std::runtime_error("The image file seems to be corrupted : " + path);
#else
    throw std::runtime_error("No codec to decode the image. Please recompile with ffmpeg");
#endif

}


CML::GrayImage CML::loadGrayImage(std::string path) {

#if CML_HAVE_AVFORMAT
    logger.debug("Load image " + path);

    FFMPEGContext ctx;

    // Open video file
    int errorCode = avformat_open_input(&ctx.mFormatCtx, path.c_str(), nullptr, nullptr);
    if(errorCode != 0) {
        throw std::runtime_error("Can't open '" + path + ". " + av_strerror(errorCode));
    }

    // Retrieve stream information
    if(avformat_find_stream_info(ctx.mFormatCtx, nullptr)<0) {
        throw std::runtime_error("Could not find stream information for '" + path + "'");
    }

    //av_dump_format(mFormatCtx, 0, path.c_str(), 0);

    // Find the first video stream
    for(unsigned int i = 0; i<ctx.mFormatCtx->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = ctx.mFormatCtx->streams[i]->codecpar;
        AVCodec *pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);


        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            ctx.mVideoStream = i;
            ctx.mCodec = pLocalCodec;
            ctx.mCodecParameters = pLocalCodecParameters;
            if (ctx.mCodec != nullptr) break;
        }
    }

    if(ctx.mVideoStream==-1) {
        throw std::runtime_error("Could not find any compatible video stream for '" + path + "'");
    }


    // Copy context
    ctx.mCodecCtx = avcodec_alloc_context3(ctx.mCodec);
    if (!ctx.mCodecCtx) {
        throw std::runtime_error("Couldn't allocate codec context for '" + path + "'");
    }

    if (avcodec_parameters_to_context(ctx.mCodecCtx, ctx.mCodecParameters) < 0) {
        fprintf(stderr, "Couldn't copy codec context");
        throw std::runtime_error("Couldn't copy codec context for '" + path + "'");
    }

    // Open codec
    if(avcodec_open2(ctx.mCodecCtx, ctx.mCodec, nullptr) < 0) {
        throw std::runtime_error("Couldn't open codec for '" + path + "'");
    }

    ctx.mFrame = av_frame_alloc();
    if (!ctx.mFrame) {
        throw std::runtime_error("Couldn't allocate frame for '" + path + "'");
    }

    //mPacket = av_packet_alloc();
    //if (!mPacket) {
    //    throw std::runtime_error("Couldn't allocate packet for '" + path + "'");
    // }

    AVPacket packet;

    while(av_read_frame(ctx.mFormatCtx, &packet)>=0) {

        if (packet.stream_index == ctx.mVideoStream) {

            int response = avcodec_send_packet(ctx.mCodecCtx, &packet);
            if (response < 0) {
                throw std::runtime_error("Error while sending a packet to the decoder");
            }

            while (response >= 0)
            {

                response = avcodec_receive_frame(ctx.mCodecCtx, ctx.mFrame);
                if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
                    break;
                } else if (response < 0) {
                    // std::string error = av_err2str(response);
                    throw std::runtime_error("Error while receiving a frame from the decoder");
                }

                if (response >= 0) {

                    // Create the frame
                    unsigned int width = ctx.mFrame->width;
                    unsigned int height = ctx.mFrame->height;

                    GrayImage image(width, height);
                    uint8_t *const data[8] = {(uint8_t*)image.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
                    const int stride[8] = {(int)(image.getWidth() * sizeof(uint8_t)), 0, 0, 0, 0, 0, 0, 0};

                    SwsContext *sws_ctx = sws_getContext(width, height, (AVPixelFormat)ctx.mFrame->format, width, height, AV_PIX_FMT_GRAY8, DEFAULT_RESIZE_ALGORITHM, NULL, NULL, NULL);
                    sws_scale(sws_ctx, (uint8_t const * const *)ctx.mFrame->data, ctx.mFrame->linesize, 0, height, data, stride);
                    sws_freeContext(sws_ctx);

                    av_packet_unref(&packet);

                    if (CML_IMAGE_HORIZONTALFLIP) {
                        return image.horizontalFlip();
                    } else {
                        return image;
                    }

                }

            }

        }

        av_packet_unref(&packet);

    }

    throw std::runtime_error("The image file seems to be corrupted : " + path);
#else
    throw std::runtime_error("No codec to decode the image. Please recompile with ffmpeg");
#endif


}

template <> CML::Array2D<CML::ColorRGBA> CML::Array2D<CML::ColorRGBA>::resize(int newWidth, int newHeight) const {
#if CML_HAVE_SWSCALE
    Image result(newWidth, newHeight);

    struct SwsContext *resizeContext;
    resizeContext = sws_getContext(getWidth(), getHeight(), AV_PIX_FMT_RGBA, newWidth, newHeight, AV_PIX_FMT_RGBA, DEFAULT_RESIZE_ALGORITHM, NULL, NULL, NULL);

    const uint8_t *const data1[8] = {(const uint8_t*)data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    uint8_t *const data2[8] = {(uint8_t*)result.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    const int stride1[8] = {(int)(getWidth() * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};
    const int stride2[8] = {(int)(newWidth * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};

    sws_scale(resizeContext, (const uint8_t *const *)data1, stride1, 0, getHeight(), (uint8_t *const *)data2, stride2);

    sws_freeContext(resizeContext);

    return result;
#else
    if (newWidth == getWidth() && newHeight == getHeight()) {
        return *this;
    }

    CML::Array2D<CML::ColorRGBA> result(newWidth, newHeight);

    #if CML_USE_OPENMP
    #pragma omp parallel for collapse(2) schedule(static)
    #endif
    for (int y = 0; y < newHeight; y++) {
        for (int x = 0; x < newWidth; x++) {
            result(x, y) = interpolate(Vector2f(
                    (float)x / (float)newWidth * ((float)getWidth() - 0.5f),
                    (float)y / (float)newHeight * ((float)getHeight() - 0.5f)
            ));
        }
    }

    return result;
#endif

}

template <> CML::Array2D<float> CML::Array2D<float>::resize(int newWidth, int newHeight) const {
    if (newWidth == getWidth() && newHeight == getHeight()) {
        return *this;
    }

    CML::Array2D<float> result(newWidth, newHeight);

    #if CML_USE_OPENMP
    #pragma omp parallel for collapse(2) schedule(static)
    #endif
    for (int y = 0; y < newHeight; y++) {
        for (int x = 0; x < newWidth; x++) {
            result(x, y) = interpolate(Vector2f(
                                        (float)x / (float)newWidth * ((float)getWidth() - 0.5f),
                                       (float)y / (float)newHeight * ((float)getHeight() - 0.5f)
                                        ));
        }
    }

    return result;

}


template <> CML::Array2D<unsigned char> CML::Array2D<unsigned char>::resize(int newWidth, int newHeight) const {
    if (newWidth == getWidth() && newHeight == getHeight()) {
        return *this;
    }

    CML::Array2D<unsigned char> result(newWidth, newHeight);

    #if CML_USE_OPENMP
    #pragma omp parallel for collapse(2) schedule(static)
    #endif
    for (int y = 0; y < newHeight; y++) {
        for (int x = 0; x < newWidth; x++) {
            result(x, y) = interpolate(Vector2f(
                    (float)x / (float)newWidth * ((float)getWidth() - 0.5f),
                    (float)y / (float)newHeight * ((float)getHeight() - 0.5f)
            ));
        }
    }

    return result;

}

template <> CML::Array2D<float> CML::Array2D<float>::convolution(const Array2D<float> &kernel) const {

    Array2D<float> newImage(*this);

    int KSizeX = kernel.mMatrix.rows();
    int KSizeY = kernel.mMatrix.cols();

    int limitRow = newImage.mMatrix.rows() - KSizeX;
    int limitCol = newImage.mMatrix.cols() - KSizeY;

    #if CML_USE_OPENMP
    #pragma omp parallel for collapse(2) schedule(static)
    #endif
    for ( int col = KSizeY; col < limitCol; col++ )
    {
        for ( int row = KSizeX; row < limitRow; row++ )
        {
            newImage.mMatrix(row,col) = (static_cast<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(mMatrix.block(row,col,KSizeX,KSizeY)).cwiseProduct(kernel.mMatrix)).sum();
        }
    }

    return newImage;
}
