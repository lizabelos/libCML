#ifndef CML_TUMCAPTURE_H
#define CML_TUMCAPTURE_H

#include <string>
#include <memory>
#include <thread>

#include "cml/config.h"
#include "cml/image/LookupTable.h"
#include "AbstractCapture.h"

#if CML_HAVE_LIBZIP

#include "ZipCaptureHelper.h"

namespace CML {

    class TUMCapture : public AbstractMultithreadFiniteCapture, public ZipCaptureHelper {

    public:
        TUMCapture(std::string path);

        ~TUMCapture();

        int remaining() final;

        int imageNumbers() final;

    protected:
        Ptr<CaptureImage, Nullable> multithreadNext();

    private:
        std::string mPath;

        std::vector<scalar_t> mExposures;
        std::vector<scalar_t> mTimestamps;
        std::vector<Vector3> mTranslations;
        std::vector<Eigen::Quaternion<scalar_t>> mRotations;
        bool mGoodGroundtruth;

        bool mGoodExposure;

        size_t mCurrentIndex;
        int mWidth, mHeight;

        std::vector<char> mZipBuffer;

        InternalCalibration *mCameraParameters;

        CaptureImageGenerator *mCaptureImageGenerator;

        GrayLookupTable mLookupTable;
        FloatImage *mVignette;

        unsigned char *mBuffer = nullptr;
        size_t mBufferSize = 0;

        GrayImage mTempImage;

    };

}

#endif

#endif
