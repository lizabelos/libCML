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

            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml");
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
            FloatImage image = loadTiffImage(data, size);

            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(image)
                    .setPath(getFilename(mCurrentImage))
                    .setTime((scalar_t)mCurrentImage / 10.0)
                    .setCalibration(mCameraParameters);

            mCurrentImage++;

            return imageMaker.generate();
        }

    private:
        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        int mCurrentImage = 1;


    };

}

#endif

#endif