#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/optimization/dso/DSOContext.h>
#include <cml/optimization/dso/DSOFrame.h>
#include <cml/optimization/dso/DSOPoint.h>
#include <cml/optimization/dso/DSOResidual.h>
#include <cml/optimization/dso/MatrixAccumulators.h>

namespace CML {

    namespace Optimization {

        /// \class DSOBundleAdjustment
        /// \brief Photometric bundle adjustment from Direct Sparse Odometry \cite{engel2017direct}
        ///
        /// This class iteratively optimize the photometric map, and compute the outliers.
        /// It is recommended to use it with \ref DSOTracker and \ref DSOTracer.
        class DSOBundleAdjustment : public AbstractFunction, public Parameter::Observer, public DSOContext {

        public:
            DSOBundleAdjustment(Ptr<AbstractFunction, NonNullable> parent);

            void createResidual(PFrame frame, PPoint point);

            void addPoints(Set<PPoint, Hasher> points);

            void addNewFrame(PFrame frame, int immatureGroup);

            void marginalizeFrame(PFrame frame);

            List<PFrame> marginalizeFrames();

            void tryMarginalize();

            std::string getName() final {
                return "DSO Bundle Adjustment";
            }

            bool run(bool updatePointsOnly = false);

            void marginalizePointsF();

            void computeNullspaces();

            const Set<PPoint, Hasher> &getOutliers() {
                return mOutliers;
            }

            void updateCamera(PFrame frame) {
                /*if (!have(frame)) {
                    return;
                }*/
                auto frameData = get(frame);
                frameData->setStateFromCamera(frame->getCamera(), mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());
            }

            Camera getLastOptimizedCamera(PFrame frame) {
                LockGuard lg(mLastOptimizedCameraMutex);
                if (mLastOptimizedCamera.count(frame) == 0) {
                    return frame->getCamera();
                } else {
                    return mLastOptimizedCamera[frame];
                }
            }

            HashMap<PFrame, Camera, Hasher> getLastOptimizedCameras() {
                LockGuard lg(mLastOptimizedCameraMutex);
                return mLastOptimizedCamera;
            }

            Set<PPoint, Hasher> getGoodPointsForTracking() {
                Set<PPoint, Hasher> points;
                for (auto point : getPoints()) {
                    auto ph = get(point);
                    if(ph->getLastResidual(0).first != nullptr && ph->getLastResidual(0).second == DSORES_IN) {
                        points.insert(point);
                    }
                }
                return points;
            }

            void setNumIterations(int it) {
                mNumIterations.set(it);
            }

            void setNumFrames(int n) {
                maxFrames.set(n);
            }

        protected:
            void computeAdjoints();

            void computeDelta();

            void setNewFrameEnergyThreshold(); // todo

            bool solveSystem(int iteration, double lambda);

            Vectord<Dynamic> solveLevenbergMarquardt(const Matrixd<Dynamic, Dynamic> &HL_top,
                                                    const Matrixd<Dynamic, Dynamic> &HA_top,
                                                    const Matrixd<Dynamic, Dynamic> &HM_top,
                                                    const Matrixd<Dynamic, Dynamic> &H_sc,
                                                    const Vectord<Dynamic> &bL_top,
                                                    const Vectord<Dynamic> &bA_top,
                                                    const Vectord<Dynamic> &bM_top,
                                                    const Vectord<Dynamic> &b_sc,
                                                    double lambda,
                                                    bool mustOrthogonalize);

            Vector3 linearizeAll(bool fixLinearization);

            scalar_t linearize(const Ptr<DSOResidual, NonNullable> &pair, const DSOFramePrecomputed &precomputed, int level = 0);

            void applyActiveRes(bool copyJacobians);

            void applyRes(Ptr<DSOResidual, NonNullable> residual, bool copyJacobians);

            void backupState();

            bool doStepFromBackup(bool fixCamera);

            void loadSateBackup();

            scalar_t calcMEnergy();

            scalar_t calcLEnergy();

            void fixLinearization(Ptr<DSOResidual, NonNullable> residual);

            int addToHessianTop(PPoint point, Ptr<DSOPoint, NonNullable> p, DSOResidualMode mode);

            void stitchDoubleTop(List<dso::AccumulatorApprox> &acc, Matrixd<Dynamic, Dynamic> &H, Vectord<Dynamic> &b, bool usePrior);

            void addToHessianSC(PPoint point, Ptr<DSOPoint, NonNullable> p, bool shiftPriorToZero);

            void stitchDoubleSC(Matrixd<Dynamic, Dynamic> &H, Vectord<Dynamic> &b);

            void orthogonalize(Vectord<Dynamic>& b);

            void setNewFrameEnergyTH();

            bool isOOB(PPoint p, const List<PFrame> &toKeep, const List<PFrame> &toMarg);

            void flagFramesForMarginalization(PFrame newFrame, int immatureGroup);

            void makeIDX();

            void setZero();

        private:
            void onValueChange(const Parameter &parameter) final;

            int DSOTOMARGINALIZE = getMap().createMapPointGroup("DSO To Marginalize");
            int DSOMARGINALIZED = getMap().createMapPointGroup("DSO Marginalized");

            PinholeUndistorter mPinhole;
            float mWidth, mHeight;

            List<Ptr<DSOResidual, NonNullable>> mActiveResiduals;

            List<dso::AccumulatorApprox> mAccumulatorActive, mAccumulatorLinearized;

            List<dso::AccumulatorXX<8,4>> mAccE;
            List<dso::AccumulatorX<8>> mAccEB;
            List<dso::AccumulatorXX<8,8>> mAccD;
            dso::AccumulatorXX<4,4> mAccHcc;
            dso::AccumulatorX<4> mAccbc;

            bool mIndicesValid = false;

            bool mAdjointsValid = false;
            List<Matrix<8,8>> mAdHost, mAdTarget;

            bool mDeltaValid = false;
            List<Matrixf<1,8>> mAdHTdeltaF;

            //Vector<Dynamic> mLastX;

            Vector4f mCDeltaF;
            Vector4 mCPrior;
            Vector4 mCalibStep, mCalibBackup, mCalibZero;
            bool mCalibHaveZero = false;

            List<Vector<Dynamic>> mLastNullspaces_pose;
            List<Vector<Dynamic>> mLastNullspaces_scale;
            List<Vector<Dynamic>> mLastNullspaces_affA;
            List<Vector<Dynamic>> mLastNullspaces_affB;

            Matrixd<Dynamic, Dynamic> mMarginalizedHessian;
            Vectord<Dynamic> mMarginalizedB;

            Set<PPoint, Hasher> mOutliers;

            PStatistic mStatisticEnergyP = createStatistic("P Energy ( All residuals )");
            PStatistic mStatisticEnergyR = createStatistic("R Energy");
            PStatistic mStatisticEnergyL = createStatistic("L Energy ( Linearized )");
            PStatistic mStatisticEnergyM = createStatistic("M Energy ( Marginalized )");
            PStatistic mStatisticEnergyTotal = createStatistic("Total Energy");
            PStatistic mStatisticXNorm = createStatistic("X Norm");
            PStatistic mStatisticHessianP = createStatistic(" Hessian P Norm");
            PStatistic mStatisticHessianL = createStatistic(" Hessian L Norm");
            PStatistic mStatisticHessianM = createStatistic(" Hessian M Norm");
            PStatistic mStatisticHessianSC = createStatistic(" Hessian SC Norm");
            PStatistic mStatisitcBP = createStatistic("P B Norm");
            PStatistic mStatisitcBL = createStatistic("L B Norm");
            PStatistic mStatisitcBM = createStatistic("M B Norm");
            PStatistic mStatisitcBSC = createStatistic("SC B Norm");
            PStatistic mStatisticOOB = createStatistic("OOB");
            PStatistic mStatisticIn = createStatistic("In");
            PStatistic mStatisticInIn = createStatistic("InIn");
            PStatistic mStatisticNores = createStatistic("Nores");
            PStatistic mStatisticNumLinearized = createStatistic("Num Linearized");

            Parameter mScaledVarTH = createParameter("Rel Var TH", 0.000001f);
            Parameter mAbsVarTH = createParameter("Abs Var TH", 0.000001f);
            Parameter mMinRelBS = createParameter("Min Relative BS", 0.4f);

            Parameter mNumIterations = createParameter("iterations", 6);

            Parameter mHuberThreshold = createParameter("Huber threshold", 9.0f);
            Parameter mSettingOutlierTHSumComponent = createParameter("outlierTHSumComponent", 50.0f * 50.0f);

            Parameter mThOptIterations = createParameter("ThOptIterations", 1.2f);

            Parameter mScaleRotation = createParameter("Rotation scale", 1.0f);
            Parameter mScaleTranslation = createParameter("Translation scale", 0.5f);
            Parameter mScaleLightA = createParameter("Light A scale", 10.0f);
            Parameter mScaleLightB = createParameter("Light B scale", 1000.0f);
            Parameter mScaleF = createParameter("Scale F", 50.0f);
            Parameter mScaleC = createParameter("Scale C", 50.0f);

            Parameter mForceAccept = createParameter("Force Accept", true);
            Parameter mFixLambda = createParameter("Fix lambda", true);
            Parameter mFixedLambda = createParameter("Fixed lambda", 1e-5f);

            Parameter mIdepthFixPrior = createParameter("iDepth Fix Prior", 50 * 50);

            Parameter mSolverModeDelta = createParameter("Solver mode delta", 0.00001f);

            Parameter mInitialCalibHessian = createParameter("Initial calib hessian", 5e9f);

            Parameter mMinIdepthHMarg = createParameter("Minimum iDepth Hessian Marginlaization", 50.0f);


            Parameter mSettingOutlierTH = createParameter("Outlier threshold", 12.0f * 12.0f);

            Parameter mCutoffThreshold = createParameter("Cutoff threshold", 20.0f);


            Parameter maxFrames = createParameter("maxFrames", 7);
            Parameter minFrameAge = createParameter("Minimum age for a frame", 1);

            LightModeOptimization mTrackerLightMode = OptimizeLightAB;
            LightModeOptimization mBALightMode = OptimizeLightAB;


            bool mAbortBAOnFailture = false;

            Parameter mOptimizeCalibration = createParameter("Optimize calibration", false);

            Mutex mLastOptimizedCameraMutex;
            HashMap<PFrame, Camera, Hasher> mLastOptimizedCamera;

            Atomic<scalar_t> timerA, timerB, timerC;
            Atomic<int> timerAc, timerBc, timerCc;

        };

    }

}