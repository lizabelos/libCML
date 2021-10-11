//
// Created by tbelos on 16/05/19.
//

#ifndef CML_ETH3DCAPTURE_H
#define CML_ETH3DCAPTURE_H

#include <string>
#include <vector>
#include <chrono>

#include "cml/config.h"
#include "AbstractCapture.h"
#include "cml/image/LookupTable.h"

namespace CML {

    class Eth3DCapture : public AbstractMultithreadFiniteCapture {

    public:
        Eth3DCapture() = default;
        Eth3DCapture(const std::string &path);
        ~Eth3DCapture();

        bool isInit();

        Ptr<CaptureImage, Nullable> multithreadNext() final;

        int remaining() final;

        int imageNumbers() final;

    private:
        bool mIsInit = false;

        std::chrono::time_point<std::chrono::high_resolution_clock> mStart;

        std::vector<std::string> mImages;
        std::vector<float> mTimes;

        size_t mCurrentImage = 0;

        InternalCalibration *mCalibration;

        CaptureImageGenerator *mCaptureImageGenerator;

        GrayLookupTable mLut;

        Array2D<float> mVignette;
        float mVignetteMax;

    };

}


#endif //CML_KITTYCAPTURE_H
