//
// Created by tbelos on 16/05/19.
//

#ifndef CML_StereopolisCapture_H
#define CML_StereopolisCapture_H

#include <string>
#include <vector>
#include <chrono>

#include "cml/config.h"
#include "AbstractCapture.h"
#include "cml/image/LookupTable.h"

namespace CML {

    class StereopolisCapture : public AbstractMultithreadFiniteCapture {

    public:
        StereopolisCapture() = default;
        StereopolisCapture(const std::string &path, const bool& reverse, const int & start, const double& expFactor, const int& topFactor );
        ~StereopolisCapture();

        bool isInit();
        void createPathList(std::string path);
        void createPathListAll(const std::string &path);

        Ptr<CaptureImage, Nullable> multithreadNext() final;

        int remaining() final;

    protected:
        FloatImage loadImage(int id);

    private:
        bool mIsInit = false;
        std::string mPath;
        bool mReverse = 0 ;
        int mStart = 0 ;
        std::vector<std::string> mPathList;
        std::vector<scalar_t> mTimestamps;

        float mTime = 0;
        size_t mCurrentIndex;
        int mWidth, mHeight;

        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        int mCurrentImage = 1;
        GrayImage mMask;
        GrayLookupTable mLookupTable;
        float mImageMax = 0;

    };

}


#endif //CML_StereopolisCapture_H
