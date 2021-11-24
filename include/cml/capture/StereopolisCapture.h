#ifndef CML_STEREOPOLISCAPTURE_H
#define CML_STEREOPOLISCAPTURE_H

#include "cml/config.h"

#if CML_HAVE_LIBZIP

#include "ZipCaptureHelper.h"

namespace CML {

    class StereopolisCapture : public AbstractMultithreadFiniteCapture, public ZipCaptureHelper {

    public:
        inline StereopolisCapture(const std::string &zipPath) {
            loadZip(zipPath, ".tif");

            uint8_t *data;
            size_t size;
            decompressFile(1, &data, &size);
            FloatImage image = loadTiffImage(data, size);

            mCaptureImageGenerator = new CaptureImageGenerator(image.getWidth(), image.getHeight());

            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml", mCaptureImageGenerator->getOutputSize());

            mMask = loadGrayImage(zipPath + ".mask.bmp");

            int histogram[256];
            for (int i = 0; i < 256; i++) {
                histogram[i] = 0;
            }
            for (int y = 0; y < image.getHeight(); y++) {
                for (int x = 0; x < image.getWidth(); x++) {
                    histogram[(int)image(x,y)]+=1;
                }
            }
            int threshold = (image.getWidth() * image.getHeight()) * 0.999;
            for (int i = 1; i < 256; i++) {
                histogram[i] += histogram[i - 1];
                if (histogram[i] > threshold) {
                    mImageMax = i;
                    break;
                }
            }

            mLookupTable = GrayLookupTable::exp(255, 1.005f);
        }

        inline int remaining() final {
            return imageNumbers() - mCurrentImage;
        }

        inline int imageNumbers() final {
            return getImageNumber();
        }

    protected:
        inline Ptr<CaptureImage, Nullable> multithreadNext() {
            uint8_t *data;
            size_t size;
            decompressFile(mCurrentImage, &data, &size);
            FloatImage image = loadTiffImage(data, size) * (255.0f / mImageMax);

            for (int y = 0; y < image.getHeight(); y++) {
                for (int x = 0; x < image.getWidth(); x++) {
                    if (mMask(x,y) < 128) {
                        image(x,y) = std::numeric_limits<float>::quiet_NaN();
                    }
                    else if (image(x,y) > 255.9) {
                        image(x,y) = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }

            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(image)
                    .setPath(getFilename(mCurrentImage))
                    .setTime((scalar_t)mCurrentImage / 10.0)
                    .setCalibration(mCameraParameters)
                    .setLut(&mLookupTable);

            mCurrentImage++;

            return imageMaker.generate();
        }

    private:
        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        int mCurrentImage = 1;
        GrayImage mMask;
        GrayLookupTable mLookupTable;
        float mImageMax = 0;


    };

}

#endif

#endif