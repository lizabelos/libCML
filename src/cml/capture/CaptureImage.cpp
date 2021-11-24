//
// Created by tbelos on 17/04/19.
//

#include "cml/capture/CaptureImage.h"
#include "cml/image/MatrixPool.h"
#include "cml/image/Filter.h"
#include "cml/map/InternalCalibration.h"



bool CML::CaptureImage::stillInMemory() const {
#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
    return mOriginalGrayImages[0]->getId() == mId;
#else
    return mGrayImages[0]->getId() == mId;
#endif
}

void CML::CaptureImage::makeKey() {
    mParent->makeKey(*this);
}

void CML::CaptureImage::makeUnactive() {
    mParent->makeUnactive(*this);
}

CML::CaptureImageGenerator::CaptureImageGenerator(int width, int height, int defaultPoolSize, int keyPoolSize, int pyramidSize) {

    if (pyramidSize == -1) {

        mPyramidSize = 0;

        Vector2 s(width, height);
        while (true) {
            double area = s.x() * s.y();
            if (area > 1000 * 1000) {
                s = s / SCALEFACTOR;
                continue;
            }
            if (area <= 25*25) {
                if (mPyramidSize >= 5) {
                    break;
                } else {
                    // todo
                }
            }
/*#if CML_CAPTUREIMAGE_REDUCEBYTWO && SCALEFACTOR==2
            if ((int)s(0) % 2 != 0) {
                s(0) = s(0) - 1;
            }
            if((int)s(1) % 2 != 0) {
                s(1) = s(1) - 1;
            }
#endif*/
            mWidths.emplace_back(s(0));
            mHeights.emplace_back(s(1));
            mPyramidSize++;
            s = s / SCALEFACTOR;
        }

    } else {

        mPyramidSize = pyramidSize;

    }

    if (mEnablePool) {

        mGrayDefaultMatrixPools.resize(mPyramidSize);
        mGradientDefaultMatrixPools.resize(mPyramidSize);
        mWeightedGradientDefaultMatrixPools.resize(mPyramidSize);
        //mColorKeyMatrixPools.resize(mPyramidSize);
        //mGrayKeyMatrixPools.resize(mPyramidSize);
        //mGradientKeyMatrixPools.resize(mPyramidSize);

        for (int level = 0; level < mPyramidSize; level++) {

            mGradientDefaultMatrixPools[level] = new MatrixPool<Vector3f>(mWidths[level], mHeights[level], defaultPoolSize);
            mGrayDefaultMatrixPools[level] = new MatrixPool<float>(mWidths[level], mHeights[level], defaultPoolSize);
            mWeightedGradientDefaultMatrixPools[level] = new MatrixPool<float>(mWidths[level], mHeights[level], defaultPoolSize);

        }

    }

}

CML::CaptureImageGenerator::~CaptureImageGenerator() {
    for (int level = 0; level < mPyramidSize; level++) {
        delete mColorDefaultMatrixPools[level];
        delete mGrayDefaultMatrixPools[level];
        delete mGradientDefaultMatrixPools[level];

       // delete mColorKeyMatrixPools[level];
       // delete mGrayKeyMatrixPools[level];
       // delete mGradientKeyMatrixPools[level];

    }
}

CML::Ptr<CML::CaptureImage, CML::NonNullable> CML::CaptureImageGenerator::generate(const CaptureImageMaker &captureImageMaker) {

    if (!captureImageMaker.mColorImage.has_value() && !captureImageMaker.mGrayImage.has_value()) {
        logger.fatal("Need image to generate capture image");
        abort();
    }

    CaptureImage *captureImage = new CaptureImage;
    captureImage->mParent = this;
    captureImage->mTime = captureImageMaker.mTime.valueOrAbort();
    captureImage->mWidths = mWidths;
    captureImage->mHeights = mHeights;
    captureImage->mInternalCalibration = captureImageMaker.mCalibration.valueOrAbort();
    captureImage->mGroundTruth = captureImageMaker.mGroundtruth;
    captureImage->mExposureTime = captureImageMaker.mExposure.valueOrDefault(1);
    captureImage->mPath = captureImageMaker.mPath.valueOrDefault("");

    assertThrow(captureImage->mInternalCalibration->getOutputSize().cast<int>() == getOutputSize(), "Invalid calibration output size");

    bool enableColor = captureImageMaker.mColorImage.has_value();

    if (enableColor && mColorDefaultMatrixPools.size() == 0) {
        mColorDefaultMatrixPools.resize(mPyramidSize);
        for (int level = 0; level < mPyramidSize; level++) {
            mColorDefaultMatrixPools[level] = new MatrixPool<ColorRGBA>(mWidths[level], mHeights[level], mGrayDefaultMatrixPools[level]->getPoolSize());
        }
    }

    Image undistortedColorImage = captureImageMaker.mColorImage.valueOrDefault(Image());
    FloatImage undistortedGrayImage;

    int wh = 0;

    if (!captureImageMaker.mGrayImage.has_value()) {
        undistortedGrayImage = FloatImage(undistortedColorImage.getWidth(), undistortedColorImage.getHeight());
        wh = undistortedGrayImage.getWidth() * undistortedGrayImage.getHeight();
        #if CML_USE_OPENMP
        #pragma omp parallel for schedule(static)
        #endif
        if (captureImageMaker.mLut.has_value()) {
            for (int i = 0; i < wh; i++) {
                undistortedGrayImage.data()[i] = (*captureImageMaker.mLut.value())(
                        undistortedColorImage.data()[i].gray());
            }
        } else {
            for (int i = 0; i < wh; i++) {
                undistortedGrayImage.data()[i] = undistortedColorImage.data()[i].gray();
            }
        }
    } else {
        undistortedGrayImage = captureImageMaker.mGrayImage.value();
        wh = undistortedGrayImage.getWidth() * undistortedGrayImage.getHeight();
        #if CML_USE_OPENMP
        #pragma omp parallel for schedule(static)
        #endif
        if (captureImageMaker.mLut.has_value()) {
            for (int i = 0; i < wh; i++) {
                undistortedGrayImage.data()[i] = (*captureImageMaker.mLut.value())(undistortedGrayImage.data()[i]);
            }
        } else {
            for (int i = 0; i < wh; i++) {
                undistortedGrayImage.data()[i] = undistortedGrayImage.data()[i];
            }
        }
    }

    if (captureImageMaker.mInverseVignette.has_value()) {
        #if CML_USE_OPENMP
        #pragma omp parallel for schedule(static)
        #endif
        for (int i = 0; i < wh; i++) {
            undistortedGrayImage.data()[i] *= captureImageMaker.mInverseVignette.value().data()[i];
        }
    }


    if (captureImage->mInternalCalibration != nullptr) {
        if (enableColor) {
            undistortedColorImage = captureImage->mInternalCalibration->fastRemoveDistortion(undistortedColorImage, mWidths[0], mHeights[0]);
        }
        undistortedGrayImage = captureImage->mInternalCalibration->removeDistortion(undistortedGrayImage, mWidths[0], mHeights[0]);
    }


    Ptr<Image, Nullable> lastColorImage;
    Ptr<FloatImage, Nullable> lastGrayImage;
    Ptr<GradientImage, Nullable> lastGradientImage;
    Ptr<GradientImage, Nullable> lastWeightedGradientImage;

    for (int level = 0; level < mPyramidSize; level++) {

        Ptr<Image, Nullable> colorImage;
        Ptr<FloatImage, Nullable> grayImage;
        Ptr<GradientImage, Nullable> gradientImage;
        Ptr<FloatImage, Nullable> weightedGradientImage;

        if (mEnablePool) {
            if (enableColor) colorImage = mColorDefaultMatrixPools[level]->load();
            grayImage = mGrayDefaultMatrixPools[level]->load();
            gradientImage = mGradientDefaultMatrixPools[level]->load();
            weightedGradientImage = mWeightedGradientDefaultMatrixPools[level]->load();

            } else {
            if (enableColor) colorImage = new Image(mWidths[level], mHeights[level]);
            gradientImage = new GradientImage(mWidths[level], mHeights[level]);

            grayImage = new FloatImage(mWidths[level], mHeights[level]);
            weightedGradientImage = new FloatImage(mWidths[level], mHeights[level]);
        }

        if (enableColor) captureImage->mColorImages.emplace_back(colorImage);
        captureImage->mGrayImages.emplace_back(grayImage);
        captureImage->mGradientImages.emplace_back(gradientImage);
        captureImage->mGradientNormWeighted.emplace_back(weightedGradientImage);


        if (level == 0) {
            assertThrow(undistortedGrayImage.getWidth() == mWidths[0] && undistortedGrayImage.getHeight() == mHeights[0], "Invalid size");
            if (enableColor) colorImage->copyToThis(undistortedColorImage);
            grayImage->copyToThis(undistortedGrayImage);
        } else {
#if CML_CAPTUREIMAGE_REDUCEBYTWO && SCALEFACTOR==2
            if (enableColor) colorImage->copyToThis(lastColorImage->fastReduceByTwo());
            grayImage->copyToThis(lastGrayImage->reduceByTwo());
#else
            if (enableColor) colorImage->copyToThis(lastColorImage->resize(mWidths[level], mHeights[level]));
            grayImage->copyToThis(lastGrayImage->resize(mWidths[level], mHeights[level]));
#endif
        }

        if (enableColor) lastColorImage = colorImage;
        lastGrayImage = grayImage;

        //gradientImage->copyToThis(grayImage->gradientImage());
        grayImage->gradientImage(*gradientImage.p());
        //weightedGradientImage->copyToThis(FloatImage::from(WeightedGradientImageProxyFloat(*gradientImage.p(), *grayImage.p(), captureImageMaker.mLut.valueOrAbort())));
        if (captureImageMaker.mLut.has_value()) {
            FloatImage::from(WeightedGradientImageProxy(*gradientImage.p(), captureImageMaker.mLut.value()), *weightedGradientImage.p());
        } else {
            FloatImage::from(WeightedGradientImageProxy(*gradientImage.p(), &mDefaultLookupTable), *weightedGradientImage.p());
        }

    }

    captureImage->mId = captureImage->mGrayImages[0]->getId();
    return captureImage;

}

void CML::CaptureImageGenerator::makeKey(CaptureImage &cf) {

    if (cf.mKeypool) {
        logger.error("The capture image is already in the keyframe pool !");
        return;
    }

    if (mEnablePool) {

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
        List<Ptr<Image, NonNullable>> originalColorImages;
        List<Ptr<GrayImage, NonNullable>> originalGrayImages;
        List<Ptr<Array2D<GrayPixelWithGradient>, NonNullable>> originalGradientImages;

        for (int level = 0; level < mPyramidSize; level++) {

            Ptr<Image, Nullable> colorImage;
            Ptr<GrayImage , Nullable> grayImage;
            Ptr<GradientImage , Nullable> gradientImage;

            colorImage = new Image(mWidths[level], mHeights[level]);
            grayImage = new GrayImage(mWidths[level], mHeights[level]);
            gradientImage = new GradientImage(mWidths[level], mHeights[level]);

            originalColorImages.emplace_back(colorImage);
            originalGrayImages.emplace_back(grayImage);
            originalGradientImages.emplace_back(gradientImage);

            originalColorImages[level]->copyToThis(*cf.mOriginalColorImages[level].p());
            originalGrayImages[level]->copyToThis(*cf.mOriginalGrayImages[level].p());
            originalGradientImages[level]->copyToThis(*cf.mOriginalGradientImages[level].p());
        }

        cf.mOriginalColorImagesBak = cf.mOriginalColorImages;
        cf.mOriginalGrayImagesBak = cf.mOriginalGrayImages;
        cf.mOriginalGradientImagesBak = cf.mOriginalGradientImages;

        cf.mOriginalColorImages = originalColorImages;
        cf.mOriginalGrayImages = originalGrayImages;
        cf.mOriginalGradientImages = originalGradientImages;

        for (int level = 0; level < mPyramidSize; level++) {
            cf.mColorImages[level]->changeChildren(cf.mOriginalColorImages[level].p());
            cf.mGrayImages[level]->changeChildren(cf.mOriginalGrayImages[level].p());
            cf.mGradientNormWeighted[level]->changeChildren(cf.mOriginalGradientImages[level].p());
        }

        cf.mId = cf.mOriginalGrayImages[0]->getId();
        cf.mKeypool = true;
#else
        List<Ptr<Image, NonNullable>> colorImages;
        List<Ptr<FloatImage, NonNullable>> grayImages;
        List<Ptr<Array2D<Vector3f>, NonNullable>> gradientImages;
        List<Ptr<FloatImage, NonNullable>> weightedGradientImages;

        for (int level = 0; level < mPyramidSize; level++) {

            Ptr<Image, Nullable> colorImage;
            Ptr<FloatImage , Nullable> grayImage;
            Ptr<GradientImage , Nullable> gradientImage;
            Ptr<FloatImage , Nullable> weightedGradientImage;

            if (cf.haveColorImage()) colorImage = new Image(mWidths[level], mHeights[level]);
            grayImage = new FloatImage(mWidths[level], mHeights[level]);
            gradientImage = new GradientImage(mWidths[level], mHeights[level]);
            weightedGradientImage = new FloatImage(mWidths[level], mHeights[level]);

            if (cf.haveColorImage()) colorImage->copyToThis(*cf.mColorImages[level].p());
            grayImage->copyToThis(*cf.mGrayImages[level].p());
            gradientImage->copyToThis(*cf.mGradientImages[level].p());
            weightedGradientImage->copyToThis(*cf.mGradientNormWeighted[level].p());

            if (cf.haveColorImage()) colorImages.emplace_back(colorImage);
            grayImages.emplace_back(grayImage);
            gradientImages.emplace_back(gradientImage);
            weightedGradientImages.emplace_back(weightedGradientImage);

        }

        cf.mColorImagesBak = cf.mColorImages;
        cf.mGrayImagesBak = cf.mGrayImages;
        cf.mGradientImagesBak = cf.mGradientImages;
        cf.mGradientNormWeightedBak = cf.mGradientNormWeighted;

        cf.mColorImages = colorImages;
        cf.mGrayImages = grayImages;
        cf.mGradientImages = gradientImages;
        cf.mGradientNormWeighted = weightedGradientImages;

        cf.mId = cf.mGrayImages[0]->getId();
        cf.mKeypool = true;
#endif
    }
}

void CML::CaptureImageGenerator::makeUnactive(CaptureImage &cf) {

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
    if (mEnablePool) {
        List<Ptr<Image, NonNullable>> originalColorImages = cf.mOriginalColorImages;
        List<Ptr<GrayImage, NonNullable>> originalGrayImages = cf.mOriginalGrayImages;
        List<Ptr<Array2D<GrayPixelWithGradient>, NonNullable>> originalGradientImages = cf.mOriginalGradientImages;

        cf.mOriginalColorImages = cf.mOriginalColorImagesBak;
        cf.mOriginalGrayImages = cf.mOriginalGrayImagesBak;
        cf.mOriginalGradientImages = cf.mOriginalGradientImagesBak;

        for (int level = 0; level < mPyramidSize; level++) {
            delete originalColorImages[level].p();
            delete originalGrayImages[level].p();
            delete originalGradientImages[level].p();
        }
    }
#else
    if (mEnablePool) {
        List<Ptr<Image, NonNullable>> colorImages = cf.mColorImages;
        List<Ptr<FloatImage, NonNullable>> grayImages = cf.mGrayImages;
        List<Ptr<Array2D<Vector3f>, NonNullable>> gradientImages = cf.mGradientImages;
        List<Ptr<FloatImage, NonNullable>> weightedGradientImages = cf.mGradientNormWeighted;

        cf.mColorImages = cf.mColorImagesBak;
        cf.mGrayImages = cf.mGrayImagesBak;
        cf.mGradientImages = cf.mGradientImagesBak;
        cf.mGradientNormWeighted = cf.mGradientNormWeightedBak;

        for (int level = 0; level < mPyramidSize; level++) {
            if (cf.haveColorImage()) delete colorImages[level].p();
            delete grayImages[level].p();
            delete gradientImages[level].p();
            delete weightedGradientImages[level].p();
        }
    }
#endif

}
