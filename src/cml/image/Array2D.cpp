#include <tiffio.h>
#include <tiffio.hxx>

#include "cml/image/Array2D.h"

#ifdef WIN32
#define HAVE_BOOLEAN
#endif

#include <thirdparty/gdcmjpeg/8/jpeglib.h>
#include "lodepng/lodepng.h"

namespace CML {

    Atomic<size_t> __array2DCounter = 0;

}


template<>
CML::Array2D<CML::ColorRGBA> CML::Array2D<CML::ColorRGBA>::resize(int newWidth, int newHeight) const {
    if (newWidth == getWidth() && newHeight == getHeight()) {
        return *this;
    }

    CML::Array2D<CML::ColorRGBA> result(newWidth, newHeight);

#if CML_USE_OPENMP
#pragma omp  for collapse(2) schedule(static)
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

template<>
CML::Array2D<float> CML::Array2D<float>::resize(int newWidth, int newHeight) const {
    if (newWidth == getWidth() && newHeight == getHeight()) {
        return *this;
    }

    CML::Array2D<float> result(newWidth, newHeight);

#if CML_USE_OPENMP
#pragma omp  for collapse(2) schedule(static)
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

    float xFactor = (float)(getWidth() - 0.5f) / (float)newWidth;
    float yFactor = (float)(getHeight() - 0.5f) / (float)newHeight;

#if CML_USE_OPENMP
#pragma omp  for collapse(2) schedule(static)
#endif
    for (int y = 0; y < newHeight; y++) {
        for (int x = 0; x < newWidth; x++) {
            result(x, y) = interpolate(Vector2f(
                    (float) x * xFactor,
                    (float) y * yFactor
            ));
        }
    }

    return result;

}

template<>
void CML::Array2D<unsigned char>::resize(int newWidth, int newHeight, Array2D<unsigned char> &result) const {
#pragma omp single
    {
        if (result.getWidth() != newWidth || result.getHeight() != newHeight) {
            result = CML::Array2D<unsigned char>(newWidth, newHeight);
        }
    }

    float xFactor = (float)(getWidth() - 0.5f) / (float)newWidth;
    float yFactor = (float)(getHeight() - 0.5f) / (float)newHeight;

#if CML_USE_OPENMP
#pragma omp  for collapse(2) schedule(static)
#endif
    for (int y = 0; y < newHeight; y++) {
        for (int x = 0; x < newWidth; x++) {
            result(x, y) = interpolate(Vector2f(
                    (float) x * xFactor,
                    (float) y * yFactor
            ));
        }
    }
}

namespace CML {
    template<typename T, int bitspersample>
    CML::Pair<CML::FloatImage, CML::Image> loadTiffImage(TIFF *tif, uint32_t width, uint32_t height) {
        FloatImage image(width, height);
        Image colorImage(width, height);

        tdata_t buf = _TIFFmalloc(TIFFScanlineSize(tif));
        if (!buf) {
            throw std::runtime_error("tiff can't malloc ?");
        }
        T *bufT = (T *) buf;

        float factor = 255.0f / (float) pow(2, bitspersample);
        float factor85 = 85.0f / (float) pow(2, bitspersample);


        for (uint32_t y = 0; y < height; y++) {
            TIFFReadScanline(tif, buf, y);

            for (uint32_t x = 0; x < width; x++) {

                colorImage(x, y).g() = (float)bufT[x * 3 + 0] * factor;
                colorImage(x, y).b() = (float)bufT[x * 3 + 1] * factor;
                colorImage(x, y).r() = (float)bufT[x * 3 + 2] * factor;

                image(x, y) = (float)(bufT[x * 3 + 0] + bufT[x * 3 + 1] + bufT[x * 3 + 2]) * factor85;
                     /*   GrayLookupTable::gammaEncode(
                                (GrayLookupTable::gammaDecode((float)bufT[x * 3 + 0] * factor) +
                            GrayLookupTable::gammaDecode((float)bufT[x * 3 + 1] * factor) +
                            GrayLookupTable::gammaDecode((float)bufT[x * 3 + 2] * factor))
                            / 3.0f
                        );*/


            }
        }


        _TIFFfree(buf);
        TIFFClose(tif);

        return {image, colorImage};
    }
}

CML::Pair<CML::FloatImage, CML::Image> CML::loadTiffImage(const uint8_t *str, size_t lenght) {
    std::istringstream input_TIFF_stream(std::string((char *)str, lenght));

//Populate input_TIFF_stream with TIFF image data
//...

    TIFF *tif = TIFFStreamOpen("MemTIFF", &input_TIFF_stream);

    if (!tif) {
        throw std::runtime_error("Can't open the tiff file");
    }

    uint32_t height, width, bitspersample;
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &bitspersample);

    if (bitspersample == 8) {
        return loadTiffImage<uint8_t, 8>(tif, width, height);
    }
    if (bitspersample == 16) {
        return loadTiffImage<uint16_t, 16>(tif, width, height);
    }
    if (bitspersample == 32) {
        return loadTiffImage<uint32_t, 32>(tif, width, height);
    }
    if (bitspersample == 64) {
        return loadTiffImage<uint64_t, 64>(tif, width, height);
    }

    throw std::runtime_error("Unsupported tiff bit per sample : " + std::to_string(bitspersample));

}

/// JPEG source manager
struct jpeg_source_t
{
    struct jpeg_source_mgr _pub;

    const char *_begin;
    const char *_end;
};

/// Initialize JPEG source
METHODDEF(void) init_source(j_decompress_ptr cinfo)
{
}

/// Fill image buffer
METHODDEF(bool) fill_input_buffer(j_decompress_ptr cinfo)
{
    struct jpeg_source_t *src=(struct jpeg_source_t *)cinfo->src;

    src->_pub.next_input_byte=(const JOCTET *)src->_begin;
    src->_pub.bytes_in_buffer=src->_end-src->_begin;

    return true;
}


/// Skip data in buffer
METHODDEF(void)skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    struct jpeg_source_t *src=(struct jpeg_source_t *)cinfo->src;

    src->_pub.next_input_byte+=(size_t)num_bytes;
    src->_pub.bytes_in_buffer-=(size_t)num_bytes;
}


/// Close buffer
METHODDEF(void)term_source(j_decompress_ptr cinfo)
{
}


/// Initialize memory source
void jpeg_memory_src(j_decompress_ptr cinfo, const char *begin, const char *end)
{
    cinfo->src=(struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr)cinfo, JPOOL_PERMANENT, sizeof(struct jpeg_source_t));
    struct jpeg_source_t *src=(struct jpeg_source_t *)cinfo->src;

    src->_pub.init_source=init_source;
    src->_pub.fill_input_buffer=fill_input_buffer;
    src->_pub.skip_input_data=skip_input_data;
    src->_pub.resync_to_restart=jpeg_resync_to_restart;
    src->_pub.term_source=term_source;
    src->_pub.bytes_in_buffer=0;
    src->_pub.next_input_byte=NULL;
    src->_begin=begin;
    src->_end=end;
}

CML::Pair<CML::FloatImage, CML::Image> CML::loadJpegImage(const uint8_t *str, size_t lenght) {
/* This struct contains the JPEG decompression parameters and pointers to
   * working space (which is allocated as needed by the JPEG library).
   */
    struct jpeg_decompress_struct cinfo;
    /* We use our private extension JPEG error handler.
     * Note that this struct must live as long as the main JPEG parameter
     * struct, to avoid dangling-pointer problems.
     */
    /* More stuff */
    JSAMPARRAY buffer;    /* Output row buffer */
    int row_stride;    /* physical row width in output buffer */

    memset(&cinfo, 0, sizeof(cinfo));

    /* Step 1: allocate and initialize JPEG decompression object */

    jpeg_create_decompress(&cinfo);

    /* Step 2: specify data source  */
    jpeg_memory_src(&cinfo, reinterpret_cast<const char *>(str), reinterpret_cast<const char *>(str + lenght));

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

    CML::FloatImage resultFloat(cinfo.output_width, cinfo.output_height);
    CML::Image resultColor(cinfo.output_width, cinfo.output_height);
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
                int row = cinfo.output_scanline;
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, row) = (float)buffer[0][i] / 255.0f;
                    resultColor(i, row) = buffer[0][i];
                }
            }
            break;
        case 3:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                int row = cinfo.output_scanline;
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, row) =
                            (float) (((unsigned char *) buffer[0])[i * 3] + ((unsigned char *) buffer[0])[i * 3 + 1] +
                                     ((unsigned char *) buffer[0])[i * 3 + 2]) / 3;
                    resultColor(i, row) = ((unsigned char *) buffer[0])[i * 3];
                }
            }
            break;
        case 4:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                int row = cinfo.output_scanline;
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, row) =
                            (float) (((unsigned char *) buffer[0])[i * 4] + ((unsigned char *) buffer[0])[i * 4 + 1] +
                                     ((unsigned char *) buffer[0])[i * 4 + 2]) / 3;
                    resultColor(i, row) = ((unsigned char *) buffer[0])[i * 4];
                }
            }
            break;
        default:
            while (cinfo.output_scanline < cinfo.output_height) {
                /* jpeg_read_scanlines expects an array of pointers to scanlines.
                 * Here the array is only one element long, but you could ask for
                 * more than one scanline at a time if that's more convenient.
                 */
                int row = cinfo.output_scanline;
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);
                /* Assume put_scanline_someplace wants a pointer and sample count. */
                for (int i = 0; i < cinfo.output_width; i++) {
                    resultFloat(i, row) = ((unsigned char *) buffer[0])[i * cinfo.num_components];
                    resultColor(i, row) = ((unsigned char *) buffer[0])[i * cinfo.num_components];
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

    /* And we're done! */
    return {resultFloat, resultColor};
}


CML::Pair<CML::FloatImage, CML::Image> CML::loadPngImage(const std::string &path) {
    std::vector<unsigned char> image; //the raw pixels
    unsigned width, height;

    //decode
    unsigned error = lodepng::decode(image, width, height, path.c_str());

    //if there's an error, display it
    if (error) {
        throw std::runtime_error("Decode error : " + std::string(lodepng_error_text(error)));
    }

    //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
    CML::Image colorImage(width, height);
    memcpy(colorImage.data(), image.data(), width * height * 4);

    CML::FloatImage grayImage = colorImage.toGrayImage();

    return {grayImage, colorImage};

}