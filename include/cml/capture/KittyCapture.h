#ifndef CML_KITTYCAPTURE_H
#define CML_KITTYCAPTURE_H

#include <string>
#include <vector>
#include <chrono>

#include "cml/config.h"
#include "AbstractCapture.h"
#include "cml/image/LookupTable.h"

namespace CML {

    class KittyPose {

    public:
        Matrix44 transformMatrix;
    };

    class KittyCapture : public AbstractMultithreadFiniteCapture {

    public:
        KittyCapture() = default;
        KittyCapture(const std::string &path, bool useColor = true);
        ~KittyCapture();

        bool isInit();

        Ptr<CaptureImage, Nullable> multithreadNext() final;

        int remaining() final;

        int imageNumbers() final;

    private:
        bool mIsInit = false;
        bool mUseColor;

        std::chrono::time_point<std::chrono::high_resolution_clock> mStart;

        std::vector<std::string> mImages[4];
        std::vector<float> mTimes;
        std::vector<KittyPose> mPoses;

        size_t mCurrentImage = 0;

        InternalCalibration *mCalibration[4];

        CaptureImageGenerator *mCaptureImageGenerator;

        GrayLookupTable mLut;

        Array2D<float> mVignette;
        float mVignetteMax;

    };

}


#endif //CML_KITTYCAPTURE_H
