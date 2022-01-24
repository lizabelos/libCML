#include <tiffio.h>
#include <tiffio.hxx>

#include "cml/image/Array2D.h"

#ifdef WIN32
#define HAVE_BOOLEAN
#endif
#include <thirdparty/gdcmjpeg/8/jpeglib.h>
#include "lodepng/lodepng.h"


#if CML_HAVE_AVFORMAT
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}
#define DEFAULT_RESIZE_ALGORITHM SWS_BICUBIC
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
    // logger.debug("Load image " + path);

    FFMPEGContext ctx;

    // Open video file
    int errorCode = avformat_open_input(&ctx.mFormatCtx, path.c_str(), nullptr, nullptr);
    if (errorCode != 0) {
        throw std::runtime_error("Can't open '" + path + ". " + av_strerror(errorCode));
    }

    // Retrieve stream information
    if (avformat_find_stream_info(ctx.mFormatCtx, nullptr) < 0) {
        throw std::runtime_error("Could not find stream information for '" + path + "'");
    }

    //av_dump_format(mFormatCtx, 0, path.c_str(), 0);

    // Find the first video stream
    for (unsigned int i = 0; i < ctx.mFormatCtx->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = ctx.mFormatCtx->streams[i]->codecpar;
        AVCodec *pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);


        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            ctx.mVideoStream = i;
            ctx.mCodec = pLocalCodec;
            ctx.mCodecParameters = pLocalCodecParameters;
            if (ctx.mCodec != nullptr) break;
        }
    }

    if (ctx.mVideoStream == -1) {
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
    if (avcodec_open2(ctx.mCodecCtx, ctx.mCodec, nullptr) < 0) {
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

    while (av_read_frame(ctx.mFormatCtx, &packet) >= 0) {

        if (packet.stream_index == ctx.mVideoStream) {

            int response = avcodec_send_packet(ctx.mCodecCtx, &packet);
            if (response < 0) {
                throw std::runtime_error("Error while sending a packet to the decoder");
            }

            while (response >= 0) {

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

                    uint8_t *const data[8] = {(uint8_t *) image.data(), nullptr, nullptr, nullptr, nullptr, nullptr,
                                              nullptr};
                    const int stride[8] = {(int) (width * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};

                    SwsContext *sws_ctx = sws_getContext(width, height, (AVPixelFormat) ctx.mFrame->format, width,
                                                         height, AV_PIX_FMT_RGBA, DEFAULT_RESIZE_ALGORITHM, NULL, NULL,
                                                         NULL);
                    sws_scale(sws_ctx, (uint8_t const *const *) ctx.mFrame->data, ctx.mFrame->linesize, 0, height, data,
                              stride);
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
    // logger.debug("Load image " + path);

    FFMPEGContext ctx;

    // Open video file
    int errorCode = avformat_open_input(&ctx.mFormatCtx, path.c_str(), nullptr, nullptr);
    if (errorCode != 0) {
        throw std::runtime_error("Can't open '" + path + ". " + av_strerror(errorCode));
    }

    // Retrieve stream information
    if (avformat_find_stream_info(ctx.mFormatCtx, nullptr) < 0) {
        throw std::runtime_error("Could not find stream information for '" + path + "'");
    }

    //av_dump_format(mFormatCtx, 0, path.c_str(), 0);

    // Find the first video stream
    for (unsigned int i = 0; i < ctx.mFormatCtx->nb_streams; i++) {
        AVCodecParameters *pLocalCodecParameters = ctx.mFormatCtx->streams[i]->codecpar;
        AVCodec *pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);


        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            ctx.mVideoStream = i;
            ctx.mCodec = pLocalCodec;
            ctx.mCodecParameters = pLocalCodecParameters;
            if (ctx.mCodec != nullptr) break;
        }
    }

    if (ctx.mVideoStream == -1) {
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
    if (avcodec_open2(ctx.mCodecCtx, ctx.mCodec, nullptr) < 0) {
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

    while (av_read_frame(ctx.mFormatCtx, &packet) >= 0) {

        if (packet.stream_index == ctx.mVideoStream) {

            int response = avcodec_send_packet(ctx.mCodecCtx, &packet);
            if (response < 0) {
                throw std::runtime_error("Error while sending a packet to the decoder");
            }

            while (response >= 0) {

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
                    uint8_t *const data[8] = {(uint8_t *) image.data(), nullptr, nullptr, nullptr, nullptr, nullptr,
                                              nullptr};
                    const int stride[8] = {(int) (image.getWidth() * sizeof(uint8_t)), 0, 0, 0, 0, 0, 0, 0};

                    SwsContext *sws_ctx = sws_getContext(width, height, (AVPixelFormat) ctx.mFrame->format, width,
                                                         height, AV_PIX_FMT_GRAY8, DEFAULT_RESIZE_ALGORITHM, NULL, NULL,
                                                         NULL);
                    sws_scale(sws_ctx, (uint8_t const *const *) ctx.mFrame->data, ctx.mFrame->linesize, 0, height, data,
                              stride);
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

template<>
CML::Array2D<CML::ColorRGBA> CML::Array2D<CML::ColorRGBA>::resize(int newWidth, int newHeight) const {
#if CML_HAVE_SWSCALE
    Image result(newWidth, newHeight);

    struct SwsContext *resizeContext;
    resizeContext = sws_getContext(getWidth(), getHeight(), AV_PIX_FMT_RGBA, newWidth, newHeight, AV_PIX_FMT_RGBA,
                                   DEFAULT_RESIZE_ALGORITHM, NULL, NULL, NULL);

    const uint8_t *const data1[8] = {(const uint8_t *) data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    uint8_t *const data2[8] = {(uint8_t *) result.data(), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

    const int stride1[8] = {(int) (getWidth() * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};
    const int stride2[8] = {(int) (newWidth * sizeof(uint8_t) * 4), 0, 0, 0, 0, 0, 0, 0};

    sws_scale(resizeContext, (const uint8_t *const *) data1, stride1, 0, getHeight(), (uint8_t *const *) data2,
              stride2);

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

template<>
CML::Array2D<float> CML::Array2D<float>::resize(int newWidth, int newHeight) const {
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
                    (float) x / (float) newWidth * ((float) getWidth() - 0.5f),
                    (float) y / (float) newHeight * ((float) getHeight() - 0.5f)
            ));
        }
    }

    return result;

}


template<>
CML::Array2D<unsigned char> CML::Array2D<unsigned char>::resize(int newWidth, int newHeight) const {
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
                    (float) x / (float) newWidth * ((float) getWidth() - 0.5f),
                    (float) y / (float) newHeight * ((float) getHeight() - 0.5f)
            ));
        }
    }

    return result;

}


template<>
CML::Array2D<float> CML::Array2D<float>::convolution(const Array2D<float> &kernel, bool oldVersion) const {

    if (!oldVersion) {

        Array2D<float> newImage(*this);

        const int res_shiftx = (kernel.getWidth() - 1) / 2;
        const int res_shifty = (kernel.getHeight() - 1) / 2;
        //const int res_shiftx = 0;
        //const int res_shifty = 0;

        float tmp[4] __attribute__ ((aligned (16)));
        __m128 mkernel[kernel.getHeight()][kernel.getWidth()]  __attribute__ ((aligned (16)));
        for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
            for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                tmp[0] = kernel(ker_x, ker_y);
                tmp[1] = tmp[0];
                tmp[2] = tmp[0];
                tmp[3] = tmp[0];
                mkernel[ker_y][ker_x] = _mm_load_ps(tmp);
            }
        }

#pragma omp for schedule(static)
        for (int img_y = 0; img_y < getHeight() - kernel.getHeight(); img_y++) {
            int img_x;
            for (img_x = 0; img_x < (getWidth() - kernel.getWidth()) - 4; img_x = img_x + 4) {

                __m128 accumulation  __attribute__ ((aligned (16)));
                __m128 datablock  __attribute__ ((aligned (16)));
                float tmp[4] __attribute__ ((aligned (16)));

                accumulation = _mm_setzero_ps();
                for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
                    for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                        datablock = _mm_loadu_ps(&mMatrix(img_x + ker_x, img_y + ker_y));
                        accumulation = _mm_add_ps(_mm_mul_ps(mkernel[ker_y][ker_x], datablock), accumulation);
                    }
                }

                _mm_store_ps(tmp, accumulation);
                newImage(img_x + res_shiftx + 0, img_y + res_shifty) = tmp[0];
                newImage(img_x + res_shiftx + 1, img_y + res_shifty) = tmp[1];
                newImage(img_x + res_shiftx + 2, img_y + res_shifty) = tmp[2];
                newImage(img_x + res_shiftx + 3, img_y + res_shifty) = tmp[3];

            }

            for (img_x = -res_shiftx; img_x < res_shiftx; img_x++) {

                float accumulation = 0;
                for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
                    for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                        accumulation = accumulation + (kernel.get(ker_x, ker_y) * getBorder(img_x + ker_x, img_y + ker_y));
                    }
                }
                newImage(img_x + res_shiftx, img_y + res_shifty) = accumulation;

            }

            for (img_x = (getWidth() - kernel.getWidth()) - 4; img_x + res_shiftx < getWidth(); img_x++) {

                float accumulation = 0;
                for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
                    for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                        accumulation = accumulation + (kernel.get(ker_x, ker_y) * getBorder(img_x + ker_x, img_y + ker_y));
                    }
                }
                newImage(img_x + res_shiftx, img_y + res_shifty) = accumulation;

            }

        }

        for (int img_y = -res_shifty; img_y < res_shifty; img_y++) {
            int img_x;
            for (img_x = -res_shiftx; img_x + res_shiftx < getWidth(); img_x++) {

                float accumulation = 0;
                for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
                    for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                        accumulation = accumulation + (kernel.get(ker_x, ker_y) * getBorder(img_x + ker_x, img_y + ker_y));
                    }
                }
                newImage(img_x + res_shiftx, img_y + res_shifty) = accumulation;

            }
        }

        for (int img_y = getHeight() - kernel.getHeight(); img_y + res_shifty < getHeight(); img_y++) {
            int img_x;
            for (img_x = -res_shiftx; img_x + res_shiftx < getWidth(); img_x++) {

                float accumulation = 0;
                for (int ker_y = 0; ker_y < kernel.getHeight(); ker_y++) {
                    for (int ker_x = 0; ker_x < kernel.getWidth(); ker_x++) {
                        accumulation = accumulation + (kernel.get(ker_x, ker_y) * getBorder(img_x + ker_x, img_y + ker_y));
                    }
                }
                newImage(img_x + res_shiftx, img_y + res_shifty) = accumulation;

            }
        }

        return newImage;
    } else {

        Array2D<float> newImage(*this);

        int KSizeX = kernel.mMatrix.rows();
        int KSizeY = kernel.mMatrix.cols();

        int limitRow = newImage.mMatrix.rows() - KSizeX;
        int limitCol = newImage.mMatrix.cols() - KSizeY;

        for (int col = KSizeY; col < limitCol; col++) {
            for (int row = KSizeX; row < limitRow; row++) {
                newImage.mMatrix(row,
                                 col) = (static_cast<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(mMatrix.block(
                        row, col, KSizeX, KSizeY)).cwiseProduct(kernel.mMatrix)).sum();
            }
        }

        return newImage;
    }
}

CML::Pair<CML::FloatImage, CML::Image> CML::loadTiffImage(const uint8_t *str, size_t lenght) {
    std::istringstream input_TIFF_stream(std::string((char*)str, lenght));

//Populate input_TIFF_stream with TIFF image data
//...

    TIFF* tif = TIFFStreamOpen("MemTIFF", &input_TIFF_stream);

    if (!tif) {
        throw std::runtime_error("Can't open the tiff file");
    }

    uint32 width, height, bitspersample, depth;

    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bitspersample);
    //TIFFGetField(tif, TIFFTAG_IMAGEDEPTH, &depth);
    depth = 3;

    FloatImage image(width, height);
    Image colorImage(width, height);

    tdata_t buf = _TIFFmalloc(TIFFScanlineSize(tif));
    if (!buf) {
        throw std::runtime_error("tiff can't malloc ?");
    }
    uint8_t *buf8 = (uint8_t*)buf;

    uint32_t mask = 0;
    for (uint32_t i = 0; i < bitspersample; i++) {
        mask ^= 1U << i;
    }

    float factor = 255.0f / (float)pow(2, bitspersample);

    for (uint32_t y = 0; y < height; y++) {
        TIFFReadScanline(tif, buf, y);
        uint32_t currentBitPosition = 0;

        for (uint32_t x = 0; x < width; x++) {

            float avg = 0;

            for (uint32_t c = 0; c < depth; c++) {

                uint32_t pos = currentBitPosition / 8;
                uint32_t left = currentBitPosition % 8;
                uint32_t right = (32 - bitspersample) - left;

                uint32_t *pintensity = (uint32_t*)&buf8[pos];
                uint32_t intensity = *pintensity;

                intensity = intensity >> right;
                intensity = intensity & mask;

                float value = (float)intensity * factor;

                if (c == 0) {
                    colorImage(x,y).g() = value;

                } else if (c == 1) {
                    colorImage(x,y).b() = value;

                } else if (c == 2) {
                    colorImage(x,y).r() = value;

                }

                avg += value;

                currentBitPosition += bitspersample;

            }

            avg /= (float)depth;

            image(x,y)=avg;

        }
    }


    _TIFFfree(buf);
    TIFFClose(tif);

    return {image, colorImage};

}

CML::Pair<CML::FloatImage, CML::Image> CML::loadJpegImage(const std::string &path) {
/* This struct contains the JPEG decompression parameters and pointers to
   * working space (which is allocated as needed by the JPEG library).
   */
    struct jpeg_decompress_struct cinfo;
    /* We use our private extension JPEG error handler.
     * Note that this struct must live as long as the main JPEG parameter
     * struct, to avoid dangling-pointer problems.
     */
    /* More stuff */
    FILE * infile;    /* source file */
    JSAMPARRAY buffer;    /* Output row buffer */
    int row_stride;    /* physical row width in output buffer */

    /* In this example we want to open the input file before doing anything else,
     * so that the setjmp() error recovery below can assume the file is open.
     * VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
     * requires it in order to read binary files.
     */

    if ((infile = fopen(path.c_str(), "rb")) == nullptr) {
        throw std::runtime_error("Can't open " + path);
    }

    /* Step 1: allocate and initialize JPEG decompression object */

    jpeg_create_decompress(&cinfo);

    /* Step 2: specify data source (eg, a file) */

    jpeg_stdio_src(&cinfo, infile);

    /* Step 3: read file parameters with jpeg_read_header() */

    (void) jpeg_read_header(&cinfo, true);
    /* We can ignore the return value from jpeg_read_header since
     *   (a) suspension is not possible with the stdio data source, and
     *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
     * See libjpeg.doc for more info.
     */

    /* Step 4: set parameters for decompression */

    /* In this example, we don't need to change any of the defaults set by
     * jpeg_read_header(), so we do nothing here.
     */

    /* Step 5: Start decompressor */

    (void) jpeg_start_decompress(&cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* We may need to do some setup of our own at this point before reading
     * the data.  After jpeg_start_decompress() we have the correct scaled
     * output image dimensions available, as well as the output colormap
     * if we asked for color quantization.
     * In this example, we need to make an output work buffer of the right size.
     */
    /* JSAMPLEs per row in output buffer */
    row_stride = cinfo.output_width * cinfo.output_components;
    /* Make a one-row-high sample array that will go away when done with image */
    buffer = (*cinfo.mem->alloc_sarray)
            ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);

    /* Step 6: while (scan lines remain to be read) */
    /*           jpeg_read_scanlines(...); */

    CML::FloatImage resultFloat;
    CML::Image resultColor;
    /* Here we use the library's state variable cinfo.output_scanline as the
     * loop counter, so that we don't have to keep track ourselves.
     */
    switch (cinfo.num_components) {
        case 1:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i];
                    resultColor(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i];
                }
            }
            break;
        case 3:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, cinfo.output_scanline) = (float)(((unsigned char*)buffer[0])[i * 3] + ((unsigned char*)buffer[0])[i * 3 + 1] + ((unsigned char*)buffer[0])[i * 3 + 2]) / 3;
                    resultColor(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i * 3];
                }
            }
            break;
        case 4:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, cinfo.output_scanline) = (float)(((unsigned char*)buffer[0])[i * 4] + ((unsigned char*)buffer[0])[i * 4 + 1] + ((unsigned char*)buffer[0])[i * 4 + 2]) / 3;
                    resultColor(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i * 4];
                }
            }
            break;
        default:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i * cinfo.num_components];
                    resultColor(i, cinfo.output_scanline) = ((unsigned char*)buffer[0])[i * cinfo.num_components];
                }
            }
            break;
    }

    /* Step 7: Finish decompression */

    (void) jpeg_finish_decompress(&cinfo);
    /* We can ignore the return value since suspension is not possible
     * with the stdio data source.
     */

    /* Step 8: Release JPEG decompression object */

    /* This is an important step since it will release a good deal of memory. */
    jpeg_destroy_decompress(&cinfo);

    /* After finish_decompress, we can close the input file.
     * Here we postpone it until after no more JPEG errors are possible,
     * so as to simplify the setjmp error logic above.  (Actually, I don't
     * think that jpeg_destroy can do an error exit, but why assume anything...)
     */
    fclose(infile);

    /* At this point you may want to check to see whether any corrupt-data
     * warnings occurred (test whether jerr.pub.num_warnings is nonzero).
     */

    /* And we're done! */
    return {resultFloat, resultColor};
}


CML::Pair<CML::FloatImage, CML::Image> CML::loadPngImage(const std::string &path) {
    std::vector<unsigned char> image; //the raw pixels
    unsigned width, height;

    //decode
    unsigned error = lodepng::decode(image, width, height, path.c_str());

    //if there's an error, display it
    if(error) {
        throw std::runtime_error("Decode error : " + std::string(lodepng_error_text(error)));
    }

    //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
    CML::Image colorImage(width, height);
    memcpy(colorImage.data(), image.data(), width * height * 4);

    CML::FloatImage grayImage = colorImage.toGrayImage();

    return {grayImage, colorImage};

}