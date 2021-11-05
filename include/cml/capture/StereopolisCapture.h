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

            Pair<FloatImage, Image> images = decompressImage(0);

            mCaptureImageGenerator = new CaptureImageGenerator(images.first.getWidth(), images.first.getHeight());

            mCameraParameters = new InternalCalibration();
        }

        inline int remaining() final {
            return imageNumbers() - mCurrentImage;
        }

        inline int imageNumbers() final {
            return getImageNumber();
        }

    protected:
        inline Ptr<CaptureImage, Nullable> multithreadNext() {
            Pair<FloatImage, Image> images = decompressImage(mCurrentImage);

            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(images.first)
                    .setImage(images.second)
                    .setPath(getFilename(mCurrentImage))
                    .setTime((scalar_t)mCurrentImage / 10.0)
                    .setCalibration(mCameraParameters);

            mCurrentImage++;

            return imageMaker.generate();
        }

    private:
        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        int mCurrentImage = 0;


    };

}

#endif

#endif