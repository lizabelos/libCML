#ifndef CML_DSOTRACKER_H
#define CML_DSOTRACKER_H

#include <cml/config.h>
#include <cml/optimization/dso/DSOContext.h>
#include <cml/base/AbstractFunction.h>
#include <cml/optimization/dso/MatrixAccumulators.h>
#include <cml/simd/AlignedArray.h>

namespace CML {

    namespace Optimization {

        class DSOTrackerPrivateLevel {

        public:
            EIGEN_STRONG_INLINE constexpr size_t cindex(size_t index, size_t i) {
                return index * 7 + i;
            }

            EIGEN_STRONG_INLINE void resize(size_t size) {
                mBuffer.resize(size * 7);
                std::fill(mBuffer.begin(), mBuffer.end(), 0);
                mPCn = 0;
                mSize = size;
            }

            EIGEN_STRONG_INLINE void backupWeightSum() {
                for (size_t i = 0; i < mSize; i++) {
                    weightSumBak(i) = weightSum(i);
                }
            }

            EIGEN_STRONG_INLINE float &idepth(size_t index) {
                return mBuffer[cindex(index, 0)];
            }

            EIGEN_STRONG_INLINE float &weightSum(size_t index) {
                return mBuffer[cindex(index, 1)];
            }

            EIGEN_STRONG_INLINE float &weightSumBak(size_t index) {
                return mBuffer[cindex(index, 2)];
            }

            EIGEN_STRONG_INLINE float &PCu(size_t index) {
                return mBuffer[cindex(index, 3)];
            }

            EIGEN_STRONG_INLINE float &PCv(size_t index) {
                return mBuffer[cindex(index, 4)];
            }

            EIGEN_STRONG_INLINE float &PCidepth(size_t index) {
                return mBuffer[cindex(index, 5)];
            }

            EIGEN_STRONG_INLINE float &PCcolor(size_t index) {
                return mBuffer[cindex(index, 6)];
            }

            EIGEN_STRONG_INLINE float &n() {
                return mPCn;
            }

            EIGEN_STRONG_INLINE size_t size() {
                return mSize;
            }

        private:
            List<float> mBuffer;
            float mPCn;

            scalar_t mFirstRMSE = -1;
            size_t mSize = 0;
        };

        class DSOTrackerPrivate : public PrivateDataStructure {
        public:
            int mId;

            List<DSOTrackerPrivateLevel> level;

            scalar_t mFirstRMSE = -1;
        };

#define CML_DSOTRACKER_BUFFER_MAX_SIZE 307200

        class DSOTrackerContext {

        public:
            DSOTrackerContext() :
            buf(CML_DSOTRACKER_BUFFER_MAX_SIZE * 8)
            {

            }

            EIGEN_STRONG_INLINE size_t cindex(size_t index, size_t i) {
                size_t y = index / 4;
                size_t x = (index % 4) + (i * 4);
                size_t l = 8 * 4;
                return y * l + x;
            }

            EIGEN_STRONG_INLINE float &idepth(size_t index) {
                return buf[cindex(index,0)];
            }

            EIGEN_STRONG_INLINE float &u(size_t index) {
                return buf[cindex(index,1)];
            }

            EIGEN_STRONG_INLINE float &v(size_t index) {
                return buf[cindex(index,2)];
            }

            EIGEN_STRONG_INLINE float &dx(size_t index) {
                return buf[cindex(index,3)];
            }

            EIGEN_STRONG_INLINE float &dy(size_t index) {
                return buf[cindex(index,4)];
            }

            EIGEN_STRONG_INLINE float &residual(size_t index) {
                return buf[cindex(index,5)];
            }

            EIGEN_STRONG_INLINE float &weight(size_t index) {
                return buf[cindex(index,6)];
            }

            EIGEN_STRONG_INLINE float &refcolor(size_t index) {
                return buf[cindex(index,7)];
            }





            EIGEN_STRONG_INLINE M128 idepth_m128(size_t index) {
                return buf.m128(cindex(index,0));
            }

            EIGEN_STRONG_INLINE M128 u_m128(size_t index) {
                return buf.m128(cindex(index,1));
            }

            EIGEN_STRONG_INLINE M128 v_m128(size_t index) {
                return buf.m128(cindex(index,2));
            }

            EIGEN_STRONG_INLINE M128 dx_m128(size_t index) {
                return buf.m128(cindex(index,3));
            }

            EIGEN_STRONG_INLINE M128 dy_m128(size_t index) {
                return buf.m128(cindex(index,4));
            }

            EIGEN_STRONG_INLINE M128 residual_m128(size_t index) {
                return buf.m128(cindex(index,5));
            }

            EIGEN_STRONG_INLINE M128 weight_m128(size_t index) {
                return buf.m128(cindex(index,6));
            }

            EIGEN_STRONG_INLINE M128 refcolor_m128(size_t index) {
                return buf.m128(cindex(index,7));
            }



            EIGEN_STRONG_INLINE int &size() {
                return bufWarpedSize;
            }

            EIGEN_STRONG_INLINE dso::Accumulator9 &accumulator() {
                return mAccumulator;
            }

            Matrix<8, 1> jacobian;
            Matrix<8, 8> hessian;
            Matrix<8, 8> dampedHessian;
            Matrix<8, 1> increment;
            Matrix<8, 1> incrementScaled;

        private:
            AlignedArray<float, 4> buf;

            int bufWarpedSize;

            dso::Accumulator9 mAccumulator;


        };

        class DSOTracker : public AbstractFunction {

        public:
            class Residual {
            public:
                Residual() {
                    covariance.fill(999999);
                }

                List<scalar_t> E;
                List<int> numTermsInE;
                List<int> numSaturated;
                List<int> numRobust;
                Vector3 flowVector;
                //List<PPoint> inliers, outliers, oob;
                bool isCorrect = false;
                bool tooManySaturated = true;
                List<scalar_t> levelCutoffRepeat;
                Vector2 relAff;
                Vector6 covariance;

                EIGEN_STRONG_INLINE scalar_t rmse(size_t i = 0) const {
                    assertThrow(E.size() > 0, "Invalid residual");
                    assertThrow(numTermsInE[i] > 0, "Invalid residual");
                    return E[i] / (scalar_t)numTermsInE[i];
                }

                EIGEN_STRONG_INLINE scalar_t saturatedRatio(size_t i = 0) const {
                    assertThrow(numSaturated.size() > 0, "Invalid residual");
                    assertThrow(numTermsInE[i] > 0, "Invalid residual");
                    return (scalar_t)numSaturated[i] / (scalar_t)numTermsInE[i];
                }

            };

            DSOTracker(Ptr<AbstractFunction, NonNullable> parent);

            ~DSOTracker();

            bool trackWithMotionModel(PFrame frameToTrack, PFrame reference, const Camera &lastCamera, const Camera &prelast, Residual &residual) {

                residual = Residual();

                bool haveOneGood = false;
                Residual trackingResult, testTrackingResult;
                Camera camera, testCamera;
                Exposure exposure, testExposure;

                scalar_t achievedRes = std::numeric_limits<scalar_t>::max();

                {
                    LockGuard lg(mLastCameraMutex);
                    mLastTriedCamera.clear();
                    mLastOptimizedCamera.clear();
                }

                Exposure initialExposure = frameToTrack->getExposure();

                List<Camera> cameras = constantVelocityMotionModel(lastCamera, prelast);

                size_t i = 0;
                for (; i < cameras.size(); i++)
                {
                    testCamera = frameToTrack->getCamera() * cameras[i];

                    {
                        LockGuard lg(mLastCameraMutex);
                        mLastTriedCamera.emplace_back(testCamera);
                    }

                    testExposure.setParametersAndExposure(frameToTrack->getExposure());
                    testExposure.setParameters(initialExposure);

                    mLastResidual = trackingResult;
                    testTrackingResult = optimize(i, frameToTrack, reference, testCamera, testExposure, &mTrackerContext);

                    {
                        LockGuard lg(mLastCameraMutex);
                        mLastOptimizedCamera.emplace_back(testCamera);
                    }

                    if (trackingResult.tooManySaturated == true && testTrackingResult.tooManySaturated == false && testTrackingResult.isCorrect && std::isfinite(testTrackingResult.rmse())) {
                        haveOneGood = true;
                        camera = testCamera;
                        exposure.setParametersAndExposure(testExposure);
                        trackingResult = testTrackingResult;
                    }

                    // do we have a new winner?
                    if(testTrackingResult.isCorrect && std::isfinite(testTrackingResult.rmse()) && !(testTrackingResult.rmse() >= achievedRes))
                    {
                        if (trackingResult.tooManySaturated || !testTrackingResult.tooManySaturated) {
                            haveOneGood = true;
                            camera = testCamera;
                            exposure.setParametersAndExposure(testExposure);
                            trackingResult = testTrackingResult;
                        }
                    }

                    // take over achieved res (always).
                    if(haveOneGood)
                    {
                        if(testTrackingResult.numTermsInE[0] > 0 && testTrackingResult.rmse() < achievedRes) {
                            achievedRes = testTrackingResult.rmse();
                        }
                    }

                    float setting_reTrackThreshold = 1.5; // todo : incompatible with error ?
                    if(haveOneGood && achievedRes < mLastCoarseRMSE * setting_reTrackThreshold) {
                        break;
                    }

                }

                if(i != 0)
                {
                    logger.info("Re-track attempt " + std::to_string(i));
                }

                if(!haveOneGood)
                {
                    if (mFailureMode.i() == 1) {
                        logger.error("Big error ! Hope we can recover... (Mode 1)");
                        camera = frameToTrack->getCamera() * cameras[0];

                        exposure.setParametersAndExposure(frameToTrack->getExposure());
                        exposure.setParameters(initialExposure);

                        mLastResidual = trackingResult;
                        trackingResult = optimize(0, frameToTrack, reference, camera, exposure, &mTrackerContext);

                        haveOneGood = true;
                    } else if (mFailureMode.i() == 2) {
                        logger.error("Big error ! Hope we can recover... (Mode 2)");
                        camera = frameToTrack->getCamera() * cameras[0];

                        exposure.setParametersAndExposure(frameToTrack->getExposure());
                        exposure.setParameters(initialExposure);

                        mLastResidual = trackingResult;
                        trackingResult = optimize(0, frameToTrack, reference, camera, exposure, &mTrackerContext);

                        camera = frameToTrack->getCamera() * cameras[0];

                        exposure.setParametersAndExposure(frameToTrack->getExposure());
                        exposure.setParameters(initialExposure);

                        haveOneGood = true;
                    } else {
                        logger.error("Big error ! Hope we can recover... (Mode 0)");
                        return false;
                    }
                } else {

                    mLastCoarseRMSE = achievedRes;
                    if(get(reference)->mFirstRMSE < 0) {
                        get(reference)->mFirstRMSE = achievedRes;
                    }

                }

                frameToTrack->setCamera(camera);
                frameToTrack->setExposureParameters(exposure);
               // auto frameToTrackData = mContext.get(frameToTrack);
               // frameToTrackData->setEvalPT_scaled(Sophus::SE3<CML::scalar_t>(frameToTrack->getCamera().getRotationMatrix(), frameToTrack->getCamera().getTranslation()), frameToTrack->getExposure(), mContext.mScaleTranslation.f(), mContext.mScaleRotation.f(), mContext.mScaleLightA.f(), mContext.mScaleLightB.f());

                residual = trackingResult;


                mStatisticSaturated->addValue((residual.numSaturated[0] / (scalar_t)residual.numTermsInE[0]));
                mStatisticResidual->addValue(residual.E[0] / (scalar_t)residual.numTermsInE[0]);
                mStatisticLightA->addValue(exposure.getParameters()(0));
                mStatisticLightB->addValue(exposure.getParameters()(1));


                mStatisticFlowT->addValue(residual.flowVector(0));
                mStatisticFlowRT->addValue(residual.flowVector(2));


                return haveOneGood;
            }

            void addStatistic(Residual &residual, Exposure &exposure) {
                mStatisticSaturated->addValue((residual.numSaturated[0] / (scalar_t)residual.numTermsInE[0]));
                mStatisticResidual->addValue(residual.E[0] / (scalar_t)residual.numTermsInE[0]);
                mStatisticLightA->addValue(exposure.getParameters()(0));
                mStatisticLightB->addValue(exposure.getParameters()(1));


                mStatisticFlowT->addValue(residual.flowVector(0));
                mStatisticFlowRT->addValue(residual.flowVector(2));
            }

            std::string getName() final {
                return "DSO Tracker";
            }

            EIGEN_STRONG_INLINE scalar_t getFirstRMSE(PFrame reference) const {
                return get(reference)->mFirstRMSE;
            }

            EIGEN_STRONG_INLINE scalar_t getFirstRMSE(List<PFrame> references) const {
                scalar_t rmse = 0;
                for (auto reference : references) rmse += get(reference)->mFirstRMSE;
                return rmse;
            }

            void makeCoarseDepthL0(PFrame reference, Set<PPoint, Hasher> points);

            void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final;

            void viewOnReconstruction(DrawBoard &drawBoard) final;

            void free(PFrame frame, std::string reason) const {
                frame->getPrivateData().free(mFramePrivateDataInstance, reason);
            }

            OptPFrame getLastComputed() {
                return mLastComputed;
            }

            Residual optimize(Residual &lastResidual, int numTry, PFrame frameToTrack, PFrame reference, Camera &currentCamera, Exposure &currentExposure, DSOTrackerContext *trackerContext = nullptr) {
                mLastResidual = Residual();
                return optimize(numTry, frameToTrack, reference, currentCamera, currentExposure);
            }

        protected:
            Residual optimize(int numTry, PFrame frameToTrack, PFrame reference, Camera &currentCamera, Exposure &currentExposure, DSOTrackerContext *trackerContext = nullptr);

            void computeResidual(PFrame frameToTrack, DSOTrackerPrivate *mCD, Exposure mLastReferenceExposure, const SE3 &refToNew, Exposure exposure, Residual &residual, int level, scalar_t cutoffThreshold, DSOTrackerContext *trackerContext);

            void computeHessian(PFrame frameToTrack, DSOTrackerPrivate *mCD, Exposure mLastReferenceExposure, const SE3 &refToNew, Exposure exposure, int level, DSOTrackerContext *trackerContext);

            DSOTrackerPrivate *create(PFrame frame) const {
                bool isnew;
                DSOTrackerPrivate *p = frame->getPrivateData().get<DSOTrackerPrivate>(mFramePrivateDataInstance, isnew);
                assertThrow(isnew, "This frame already have tracker private data");
                return p;
            }

            DSOTrackerPrivate *get(PFrame frame) const {
                bool isnew;
                DSOTrackerPrivate *p = frame->getPrivateData().get<DSOTrackerPrivate>(mFramePrivateDataInstance, isnew);
                assertThrow(!isnew, "This frame don't have tracker private data");
                return p;
            }

        private:
            PrivateDataInstance mFramePrivateDataInstance;

            PStatistic mStatisticResidual = createStatistic("Residual");
            PStatistic mStatisticIncrementNorm = createStatistic("Increment norm");
            PStatistic mStatisticSaturated = createStatistic("Saturated ratio");
            PStatistic mStatisticFlowT = createStatistic("Flow T");
            PStatistic mStatisticFlowRT = createStatistic("Flow RT");
            PStatistic mStatisticLightA = createStatistic("Light A");
            PStatistic mStatisticLightB = createStatistic("Light B");

            Residual mLastResidual;

            DSOTrackerContext mTrackerContext;

            List<Camera> mLastTriedCamera, mLastOptimizedCamera;
            Mutex mLastCameraMutex;

            OptPFrame mLastComputed;

            scalar_t mLastCoarseRMSE = 100;


            Parameter mScaledVarTH = createParameter("Rel Var TH", 0.000001f);
            Parameter mAbsVarTH = createParameter("Abs Var TH", 0.000001f);
            Parameter mMinRelBS = createParameter("Min Relative BS", 0.4f);

            Parameter mNumIterations = createParameter("Iterations", 15);

            Parameter mHuberThreshold = createParameter("Huber threshold", 9.0f);
            Parameter mSettingOutlierTHSumComponent = createParameter("outlierTHSumComponent", 50.0f * 50.0f);

            Parameter mThOptIterations = createParameter("ThOptIterations", 1.2f);

            Parameter mScaleRotation = createParameter("Rotation scale", 1.0f);
            Parameter mScaleTranslation = createParameter("Translation scale", 0.5f);
            Parameter mScaleLightA = createParameter("Light A scale", 10.0f);
            Parameter mScaleLightB = createParameter("Light B scale", 1000.0f);
            Parameter mScaleF = createParameter("Scale F", 50.0f);
            Parameter mScaleC = createParameter("Scale C", 50.0f);

            Parameter mForceAccept = createParameter("Force Accept", false);
            Parameter mFixLambda = createParameter("Fix lambda", false);
            Parameter mFixedLambda = createParameter("Fixed lambda", 1e-5f);

            Parameter mIdepthFixPrior = createParameter("iDepth Fix Prior", 50 * 50);

            Parameter mSolverModeDelta = createParameter("Solver mode delta", 0.00001f);

            Parameter mInitialCalibHessian = createParameter("Initial calib hessian", 5e9f);

            Parameter mMinIdepthHMarg = createParameter("Minimum iDepth Hessian Marginlaization", 50.0f);


            Parameter mSettingOutlierTH = createParameter("Outlier threshold", 12.0f * 12.0f);

            Parameter mCutoffThreshold = createParameter("Cutoff threshold", 20.0f);


            Parameter maxFrames = createParameter("Maximum number of frames", 7);
            Parameter minFrameAge = createParameter("Minimum age for a frame", 1);

            Parameter mOptimizeA = createParameter("optimizeLightA", true);
            Parameter mOptimizeB = createParameter("optimizeLightB", true);

            Parameter mOptimizeCalibration = createParameter("Optimize calibration", false);

            Parameter mFailureMode = createParameter("failureMode", 0);
            Parameter mSaturatedRatioThreshold = createParameter("saturatedThreshold", 0.33);

        };

    }

}

#endif