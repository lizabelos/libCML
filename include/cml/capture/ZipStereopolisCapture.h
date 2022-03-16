#ifndef CML_ZIPSTEREOPOLISCAPTURE_H
#define CML_ZIPSTEREOPOLISCAPTURE_H

#include "cml/config.h"

#if CML_HAVE_LIBZIP

#include "ZipCaptureHelper.h"
#include "cml/image/Filter.h"

namespace CML {

    class ZipStereopolisCapture : public AbstractMultithreadFiniteCapture, public ZipCaptureHelper {

    public:
        inline ZipStereopolisCapture(const std::string &zipPath) {

            std::string extractPath = zipPath + "_uncompressed";
            if (!std::filesystem::is_directory(extractPath) || !std::filesystem::exists(extractPath)) { // Check if src folder exists
                std::filesystem::create_directory(extractPath);
            }
            loadZip(zipPath, ".tif", extractPath);

            uint8_t *data;
            size_t size;
            decompressFile(1, &data, &size);
            auto images = loadTiffImage(data, size);
            mMask = loadPngImage(zipPath + ".mask.png").first.castToUChar<unsigned char>();

            mLookupTable = GrayLookupTable::gammaDecode();

            mCaptureImageGenerator = new CaptureImageGenerator(images.first.getWidth(), images.first.getHeight());

            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml", mCaptureImageGenerator->getOutputSize());



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
            std::string decompressedFilePath = decompressFile(mCurrentImage, &data, &size);
            auto images = loadTiffImage(data, size);

           /* for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    if (mMask(x,y) < 128) {
                        images.first(x,y) = std::numeric_limits<float>::quiet_NaN();
                        images.second(x,y) = ColorRGBA(0,0,0,0);
                    }
                }
            }*/


            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(images.first)
              //      .setImage(images.second)
                    .setPath(decompressedFilePath)
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


    };

}

#endif

#endif