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
            auto images = loadTiffImage(data, size);
            mMask = loadPngImage(zipPath + ".mask.png").first.castToUChar<unsigned char>();

            int histogram[256];
            for (int i = 0; i < 256; i++) {
                histogram[i] = 0;
            }
            for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    histogram[(int)images.first(x,y)]+=1;
                }
            }
            int threshold = (images.first.getWidth() * images.first.getHeight()) * 0.999;
            for (int i = 1; i < 256; i++) {
                histogram[i] += histogram[i - 1];
                if (histogram[i] > threshold) {
                    mImageMax = i;
                    break;
                }
            }

            mLookupTable = GrayLookupTable::exp(255, 1.005f);

            images.first = images.first * (255.0f / mImageMax);

            int top = 0, bottom = images.first.getHeight() - 1;

            for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    if (mMask(x,y) < 128 || images.first(x,y) > 255.9) {
                        if (y < images.first.getHeight() / 2) {
                            top = std::max(top, y);
                        } else {
                            bottom = std::min(bottom, y);
                        }
                    }
                }
            }


            mCaptureImageGenerator = new CaptureImageGenerator(images.first.getWidth(), (bottom - top));

            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml", mCaptureImageGenerator->getOutputSize(), top, bottom);



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
            auto images = loadTiffImage(data, size);
            images.first = images.first * (255.0f / mImageMax);

            for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    if (mMask(x,y) < 128) {
                        images.first(x,y) = std::numeric_limits<float>::quiet_NaN();
                        images.second(x,y) = ColorRGBA(0,0,0,0);
                    }
            /*        else if (images.first(x,y) > 255.9) {
                        images.first(x,y) = std::numeric_limits<float>::quiet_NaN();
                        images.second(x,y) = ColorRGBA(0,0,0,0);
                    } */
                }
            }

            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(images.first)
                    .setImage(images.second)
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