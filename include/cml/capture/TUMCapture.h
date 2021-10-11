#ifndef CML_TUMCAPTURE_H
#define CML_TUMCAPTURE_H

#include <string>
#include <memory>
#include <thread>

#include "cml/config.h"
#include "cml/image/LookupTable.h"
#include "AbstractCapture.h"

#if CML_HAVE_LIBZIP

#include <zip.h>

namespace CML {

    class TUMCapture : public AbstractMultithreadFiniteCapture {

    public:
        TUMCapture(std::string path, int skipFrame = 0);

        ~TUMCapture();

        int remaining() final;

        int imageNumbers() final;

    protected:
        Ptr<CaptureImage, Nullable> multithreadNext();

        FloatImage loadImage(int id);

        void extractImage(int id);

    private:
        std::string mPath;
        zip_t *mZiparchive;
        std::vector<std::string> mZipFilePath, mExtractedFilePath;

        std::vector<Image*> mImages;
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

        int mSkipFrame;

        unsigned char *mBuffer = nullptr;
        size_t mBufferSize = 0;

        GrayImage mTempImage;

    };

}

#endif

#endif
