#include <cml/config.h>
#include <cml/optimization/dso/DSOContext.h>
#include <cml/base/AbstractFunction.h>
#include <cml/optimization/dso/MatrixAccumulators.h>
#include <cml/features/corner/PixelSelector.h>

namespace CML {

    namespace Optimization {

        class DSOInitializerPoint : public PrivateDataStructure {

        public:

            float &x() {
                return p.x();
            }

            float &y() {
                return p.y();
            }

            Vector2f p;
            Vector3f pPattern[8];
            Vector3f tempPt[8];
            float tempU[8];
            float tempV[8];
            float tempKu[8];
            float tempKv[8];
            float tempNewIdepth[8];

            float idepth = 1, idepth_new = 1;
            bool isGood = true, isGood_new;
            Vector2f energy = Vector2f::Zero(), energy_new;

            float initialiR = 1;
            float iR = 1;
            float iRSumNum;

            float lastHessian = 0, lastHessian_new = 0;

            float maxstep;

            // idx (x+y*w) of closest point one pyramid level above.
            int parent;
            float parentDist;

            float color[8]; // todo : pattern size
            Vector3f hitColors[8];

            // idx (x+y*w) of up to 10 nearest points in pixel space.
            int neighbours[10];
            float neighboursDist[10];

            float my_type;
            float outlierTH;

        };

        class DSOInitializer : public AbstractFunction {

        public:
            DSOInitializer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {

            }

            ~DSOInitializer() {
                if (mPixelSelector != nullptr) {
                    delete mPixelSelector;
                }
            }

            int tryInitialize(PFrame frameToTrack, PFrame reference);

            int tryInitialize(PFrame frameToTrack, PFrame reference, const Array2D<float> &inverseDepthMap);


            std::string getName() final {
                return "DSO Initializer";
            }

            void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final;

            void viewOnReconstruction(DrawBoard &drawBoard) final;

            inline DSOInitializer *setDensityFacctor(float value) {
                mDensityFactor.set(value);
                return this;
            }

            inline DSOInitializer *setRegulizationWeight(float value) {
                mRegulalizationWeight.set(value);
                return this;
            }

        protected:
            bool setFirst(PFrame reference);

            Vector3f calcResAndGS(int lvl, Matrixf<8,8> &H_out, Vector8f &b_out, Matrixf<8,8> &H_out_sc, Vector8f &b_out_sc, const Camera &camera, const Exposure &exposure);

            void propagateUp(int lvl);

            void propagateDown(int lvl);

            Vector3f calcEC(int lvl);

            void optReg(int lvl);

            void resetPoints(int lvl);

            void doStep(int lvl, float lambda, Vector8f inc);

            void applyStep(int lvl);

            void makeNN();

            void onInitializationSuccess();

        private:
            Features::PixelSelector *mPixelSelector = nullptr;

            const Pattern mPattern = PredefinedPattern::star8();

            int mNumPyramidLevel;

            OptPFrame mFrameToTrack, mReference;
            List<PFrame> mFrames;

            Mutex mPointsMutex;
            List<List<DSOInitializerPoint>> mPoints;

            Camera mCurrentCamera;
            Exposure mCurrentExposure;

            dso::Accumulator9 mAcc9;
            dso::Accumulator9 mAcc9SC;

            List<Vectorf<10>> mJbBuffer;
            List<Vectorf<10>> mJbBuffer_new;

            float mAlphaK;
            float mAlphaW;
            float mRegWeight;
            float mCouplingWeight;
            float mNNWeight = 0.2;

            bool mSnapped = false;

            int mFrameID;
            int mSnappedAt;

            int mSparsityFactor = 5;

            int mStepNum = 0;

            bool mSuccess = false;
            bool mIsInit = false;

            PStatistic mStatisticAlphaEnergy = createStatistic("Alpha Energy");

            Parameter mDensityFactor = createParameter("Density factor", 1.0f);
            Parameter mSettingsDesiredPointDensity = createParameter("pointDensity", 2000);
            Parameter mRegulalizationWeight = createParameter("Regularization weight", 0.8f);


            Parameter mNumIterations = createParameter("Iterations", 15);

            Parameter mHuberThreshold = createParameter("Huber threshold", 9.0f);


            Parameter mScaleRotation = createParameter("Rotation scale", 1.0f);
            Parameter mScaleTranslation = createParameter("Translation scale", 0.5f);
            Parameter mScaleLightA = createParameter("Light A scale", 10.0f);
            Parameter mScaleLightB = createParameter("Light B scale", 1000.0f);

            Parameter mForceAccept = createParameter("Force Accept", false);

            Parameter mSettingOutlierTH = createParameter("Outlier threshold", 12.0f * 12.0f);




        };

    }

}