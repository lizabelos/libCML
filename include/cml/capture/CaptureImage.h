//
// Created by tbelos on 17/04/19.
//

#ifndef CML_CAPTUREIMAGE_H
#define CML_CAPTUREIMAGE_H

#include <cml/config.h>
#include <cml/map/Camera.h>
#include <cml/evaluation/Evaluation.h>
#include <cml/image/Array2DProxy.h>

namespace CML {

#define CML_CAPTUREIMAGE_REDUCEBYTWO 1
#define CML_CAPTUREIMAGE_STILLINMEMORYASSERT ENABLE_ASSERTTHROW

    class CaptureImageGenerator;

    class CaptureImage : public EvaluationFrame {

    public:
        friend class CaptureImageGenerator;

        CaptureImage() {

        }

        bool stillInMemory() const;

        inline scalar_t getEvaluationTimestamp() final {
            return mTime;
        }

        inline Camera getEvaluationPosition() final {
            assertThrow(mGroundTruth.has_value(), "We don't have any groundtruth");
            return mGroundTruth.value();
        }

        inline float getTime() const {
            return mTime;
        }

        inline int getWidth(int level) const {
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
            return mWidths[level];
        }

        inline int getHeight(int level) const {
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
            return mHeights[level];
        }

        const auto &getColorImage(int level, bool checkInMemory = true) const {
            assertThrow(haveColorImage(), "Call getColorImage() but we don't have any color image");
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
#if CML_CAPTUREIMAGE_STILLINMEMORYASSERT
            if (checkInMemory) {
                if (!stillInMemory()) {
                    CML_LOG_FATAL("Image is not loaded in memory");
                    abort();
                }
            }
#endif
            return *mColorImages[level].p();
        }

        inline bool haveColorImage() const {
            return mColorImages.size() > 0;
        }

        inline const auto &getGrayImage(int level, bool checkInMemory = true) const {
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
#if CML_CAPTUREIMAGE_STILLINMEMORYASSERT
            if (checkInMemory) {
                if (!stillInMemory()) {
                    CML_LOG_FATAL("Image is not loaded in memory");
                    abort();
                }
            }
#endif
            return *mGrayImages[level].p();
        }

        inline const auto &getDerivativeImage(int level) const {
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
#if CML_CAPTUREIMAGE_STILLINMEMORYASSERT
            if (!stillInMemory()) {
                CML_LOG_FATAL("Image is not loaded in memory");
                abort();
            }
#endif
            return *mGradientImages[level].p();
        }

        inline const auto &getWeightedGradientNorm(int level) const {
            assertThrow(level < getPyramidLevels(), "Invalid level : " + std::to_string(level) + ". Have " + std::to_string(getPyramidLevels()) + " levels");
#if CML_CAPTUREIMAGE_STILLINMEMORYASSERT
            if (!stillInMemory()) {
                CML_LOG_FATAL("Image is not loaded in memory");
                abort();
            }
#endif
            return *mGradientNormWeighted[level].p();
        }

        inline InternalCalibration &getInternalCalibration() const {
            return *mInternalCalibration;
        }

        inline scalar_t getExposureTime() const {
            return mExposureTime;
        }

        inline bool hasGroundtruth() {
            return mGroundTruth.has_value();
        }

        inline Optional<Camera> getGroundtruth() {
            return mGroundTruth;
        }

        inline int getPyramidLevels() const {
            return mGrayImages.size();
        }

        inline int getNearestLevel(int w, int h) const {
            double bestDistance = std::numeric_limits<double>::max();
            int bestLevel = 0;
            for (size_t i = 0; i < mWidths.size(); i++) {
                double distance = (Vector2(w, h) - Vector2(mWidths[i], mHeights[i])).norm();
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestLevel = i;
                }
            }
            return bestLevel;
        }

        inline int getLevelWithMiniumSideSize(int w) const {
            for (size_t i = 0; i < mWidths.size(); i++) {
                if (mWidths[i] < w) return i - 1;
                if (mHeights[i] < w) return i - 1;
            }
            return getPyramidLevels();
        }

        inline CaptureImageGenerator *getGenerator() {
            return mParent;
        }

        inline scalar_t getScaleFactor(int level) {
            return pow(SCALEFACTOR, level);
        }

        inline const std::string &getPath() const {
            return mPath;
        }

        void makeKey();

        void makeUnactive();

        void postRemoveDistortion(const Array2D<Vector2f> &undistortionMap);

    protected:

        CaptureImageGenerator *mParent;

        scalar_t mTime = 0;

        List<int> mWidths, mHeights;

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
        List<Ptr<ImageProxy, NonNullable>> mColorImages;
        List<Ptr<GrayImageProxy, NonNullable>> mGrayImages;
        List<Ptr<GradientImage, NonNullable>> mGradientImages;
        List<Ptr<WeightedGradientImageProxy, NonNullable>> mGradientNormWeighted;

        List<Ptr<Array2D<ColorRGBA>, NonNullable>> mOriginalColorImages;
        List<Ptr<Array2D<uint8_t>, NonNullable>> mOriginalGrayImages;
        List<Ptr<Array2D<GrayPixelWithGradient>, NonNullable>> mOriginalGradientImages;

        List<Ptr<Array2D<ColorRGBA>, NonNullable>> mOriginalColorImagesBak;
        List<Ptr<Array2D<uint8_t>, NonNullable>> mOriginalGrayImagesBak;
        List<Ptr<Array2D<GrayPixelWithGradient>, NonNullable>> mOriginalGradientImagesBak;
#else
        List<Ptr<Image, NonNullable>> mColorImages;
        List<Ptr<FloatImage, NonNullable>> mGrayImages;
        List<Ptr<GradientImage, NonNullable>> mGradientImages;
        List<Ptr<FloatImage, NonNullable>> mGradientNormWeighted;

        List<Ptr<Image, NonNullable>> mColorImagesBak;
        List<Ptr<FloatImage, NonNullable>> mGrayImagesBak;
        List<Ptr<GradientImage, NonNullable>> mGradientImagesBak;
        List<Ptr<FloatImage, NonNullable>> mGradientNormWeightedBak;

#endif

        InternalCalibration *mInternalCalibration = nullptr;

        Optional<Camera> mGroundTruth = Optional<Camera>();

        scalar_t mExposureTime;

        size_t mId;

        std::string mPath;

        bool mKeypool = false;


    };

    class CaptureImageMaker {

        friend class CaptureImageGenerator;

    public:
        class Generator {

            friend class CaptureImageMaker;

        public:
            virtual ~Generator() {

            }

        protected:
            virtual Ptr<CaptureImage, NonNullable> generate(const CaptureImageMaker &captureImageMaker) = 0;

        };

    protected:
        inline CaptureImageMaker(Generator *generator) {
            mGenerator = generator;
        }

    public:
        inline Ptr<CaptureImage, NonNullable> generate() {
            return mGenerator->generate(*this);
        }

        inline CaptureImageMaker &setImage(const Image &image) {
            mColorImage = image;
            return *this;
        }

        inline CaptureImageMaker &setImage(const FloatImage &image, float noise = 0) {
            mGrayImage = image;
            if (noise > 0) {
                for (int i = 0; i < mGrayImage.value().getWidth() * mGrayImage.value().getHeight(); i++) {
                    float r = static_cast<float>(rand()) / static_cast <float>(RAND_MAX);
                    r = (r - 0.5f) * (noise * 2.0f);
                    mGrayImage.value().data()[i] += r;
                }
            }
            return *this;
        }

        inline CaptureImageMaker &setTime(scalar_t time) {
            mTime = time;
            return *this;
        }

        inline CaptureImageMaker &setExposure(scalar_t exposure) {
            mExposure = exposure;
            return *this;
        }

        inline CaptureImageMaker &setCalibration(InternalCalibration *calibration) {
            mCalibration = calibration;
            return *this;
        }

        inline CaptureImageMaker &setLut(GrayLookupTable *lut) {
            mLut = lut;
            return *this;
        }

        inline CaptureImageMaker &setInverseVignette(const Array2D<float> inverseVignette) {
            mInverseVignette = inverseVignette;
            return *this;
        }

        inline CaptureImageMaker &setGroundtruth(const Camera &camera) {
            mGroundtruth = camera;
            return *this;
        }

        inline CaptureImageMaker &setPath(std::string path) {
            mPath = path;
            return *this;
        }

    private:
        Generator *mGenerator;

        Optional<Image> mColorImage;
        Optional<FloatImage> mGrayImage;

        Optional<std::string> mPath;
        Optional<scalar_t> mTime;
        Optional<scalar_t> mExposure;
        Optional<InternalCalibration*> mCalibration;
        Optional<GrayLookupTable*> mLut;
        Optional<Array2D<float>> mInverseVignette;
        Optional<Camera> mGroundtruth;

    };

    class CaptureImageGenerator : public CaptureImageMaker::Generator {

        friend class CaptureImageMaker;

    public:
        CaptureImageGenerator(int width, int height, int defaultPoolSize = 64, int keyPoolSize = 64, int pyramidLevel = -1);

        ~CaptureImageGenerator() override;

        inline CaptureImageMaker create() {
            return CaptureImageMaker(this);
        }

        void makeKey(CaptureImage &captureFrame);

        void makeUnactive(CaptureImage &captureFrame);

        EIGEN_STRONG_INLINE Vector2i getOutputSize() {
            return Vector2i(mWidths[0], mHeights[0]);
        }

    protected:
        Ptr<CaptureImage, NonNullable> generate(const CaptureImageMaker &captureImageMaker) final;

    private:
        List<int> mWidths, mHeights;
        int mPyramidSize;

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
        List<MatrixPool<ColorRGBA>*> mColorDefaultMatrixPools;
        List<MatrixPool<unsigned char>*> mGrayDefaultMatrixPools;
        List<MatrixPool<GrayPixelWithGradient>*> mGradientDefaultMatrixPools;
#else
        List<MatrixPool<ColorRGBA>*> mColorDefaultMatrixPools;
        List<MatrixPool<float>*> mGrayDefaultMatrixPools;
        List<MatrixPool<Vector3f>*> mGradientDefaultMatrixPools;
        List<MatrixPool<float>*> mWeightedGradientDefaultMatrixPools;
#endif

        bool mEnablePool = true;

        GrayLookupTable mDefaultLookupTable;

    };

}


#endif //CML_CAPTUREIMAGE_H
