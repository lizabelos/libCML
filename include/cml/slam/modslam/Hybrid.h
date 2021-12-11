//
// Created by thomas on 22/02/2021.
//

#ifndef CML_HYBRID_H
#define CML_HYBRID_H

#include <thread>
#include <unistd.h>

#include <cml/base/AbstractSlam.h>

#include <cml/optimization/IndirectPointOptimizer.h>
#include <cml/optimization/dso/DSOTracer.h>
#include <cml/optimization/dso/DSOTracker.h>
#include <cml/optimization/dso/DSOBundleAdjustment.h>
#include <cml/optimization/dso/DSOInitializer.h>
#include <cml/optimization/g2o/IndirectBundleAdjustment.h>
#include <cml/optimization/g2o/IndirectCameraOptimizer.h>
#include <cml/optimization/ceres/IndirectBundleAdjustment.h>
#include <cml/optimization/EPnP.h>

#include <cml/robust/CameraChecker.h>
#include <cml/robust/RobustRaulmurInitializer.h>

#include <cml/nn/Precomputed.h>

#include <cml/features/corner/ORB.h>
#include <cml/features/cornerTracker/BoWTracker.h>
#include <cml/features/cornerTracker/RadiusTracker.h>
#include <cml/features/cornerTracker/ReprojectionTracker.h>
#include <cml/features/cornerTracker/LSHTracker.h>
#include <cml/features/cornerTracker/VFC.h>
#include <cml/features/bow/Bow.h>
#include <cml/features/bow/Relocalization.h>
#include <cml/features/corner/OpenCV.h>

#include <cml/gui/viewer/ReprojectionViewer.h>

#include <cml/maths/Utils.h>

#include <cml/utils/DistanceMap.h>

#include <cml/base/SlamThread.h>

using namespace CML;

class Hybrid : public AbstractSlam, Map::Observer, MapPoint::Observer {

private:
    using CornerAndDescriptor = Features::ORB;
    using Descriptor = CornerAndDescriptor::Descriptor;

    typedef enum BaMode {
        NOBA, BADIRECT, BAINDIRECT
    } BaMode;

    class MyFramePrivate : public PrivateDataStructure {

public:
        void reset() {
            descriptors.clear();
        }

        int featureId = -1;
        List<CornerAndDescriptor::Descriptor> descriptors;
        Mutex bowMutex;

    };

public:
    Hybrid();
    ~Hybrid();

    void viewOnReconstruction(DrawBoard &drawBoard) final {

        for (auto frame : getMap().getGroupFrames(INDIRECTKEYFRAME)) {
            drawBoard.color(1, 0, 0);
            drawBoard.lineWidth(3);
            drawBoard.paintCamera(frame->getCamera());
        }

        for (auto frame : getMap().getGroupFrames(DIRECTKEYFRAME)) {
            drawBoard.color(0, 0, 1);
            drawBoard.lineWidth(5);
            drawBoard.paintCamera(frame->getCamera());
        }

        /*for (auto point : getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT)) {
            drawBoard.color(0, 0, 1);
            drawBoard.pointSize(2);
            drawBoard.point((Vector3f)point->getWorldCoordinate().absolute().cast<float>());
        }*/

    }

    void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final {

    }

    std::string getName() final {
        return "Hybrid";
    }

    void onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) override {
        if (groupId == ACTIVEINDIRECTPOINT && state == true) {
            assertThrow(mapPoint->isGroup(getMap().INDIRECTGROUP), "Can't put a non indirect point in active indirect point");
        }
    }

protected:
    void onReset() final;
    void run() final;

private:
    void processThread();
    void processFrame(PFrame currentFrame);

    void trackWithOrbAndDsoRefinement(PFrame currentFrame);
    void trackWithDso(PFrame currentFrame);

    void indirectPostprocess(PFrame currentFrame, bool needKF);
    void directPostprocess(PFrame currentFrame, bool needKF);

    bool poseEstimationDecision();
    BaMode bundleAdjustmentDecision(bool needIndirectKF, bool needDirectKF);

    void directMap(PFrame currentFrame, bool callFromInitialization = false);
    void directMakeNonKeyFrame(PFrame currentFrame);

    void extractOrb(PFrame currentFrame);
    void needVocabularyFor(PFrame currentFrame);

    void initializeWithDSO(PFrame currentFrame);

    bool indirectTrackWithMotionModel(PFrame currentFrame, Optional<Camera> camera = Optional<Camera>());
    bool indirectTrackReferenceKeyFrame(PFrame currentFrame);
    bool indirectTrackLocalMap(PFrame currentFrame);
    void indirectSearchLocalPoints(PFrame currentFrame);
    void indirectUpdateLocalKeyFrames(PFrame currentFrame);
    void indirectUpdateLocalPoints(PFrame currentFrame);
    void indirectSearchInNeighbors(PFrame currentFrame);
    bool indirectNeedNewKeyFrame(PFrame currentFrame);
    void indirectLocalOptimize(PFrame currentFrame);
    void indirectMap(PFrame currentFrame);
    int indirectNumTrackedRef();

    List<PPoint> indirectCreateNewImmaturePointFromMatchings(const List<Matching> &matchings);
    void indirectTrackImmature(PFrame currentFrame);

   // int indirectReprojectionWithLoopClosure(PFrame currentFrame, PFrame loopClosureFrame, Optional<Camera> camera = Optional<Camera>());
    Optional<Binary256Descriptor> findDescriptor(PPoint point);


    List<PPoint> indirectCreateNewImmaturePoint(PFrame currentFrame);
    void keyframeCulling();

    bool directNeedNewKeyFrame(PFrame currentFrame);

    void directMappingLoop();


    inline Ptr<MyFramePrivate, NonNullable> get(PFrame frame) {
        bool isNew = false;
        Ptr<MyFramePrivate, NonNullable> result = frame->getPrivateData().get<MyFramePrivate>(mFramePrivateDataInstance, isNew);
        if (isNew) {
            result->descriptors.reserve(mNumOrbCorner.i());
        }
        return result;
    }

    inline bool have(PFrame frame) {
        return frame->getPrivateData().have(mFramePrivateDataInstance);
    }

    inline void freePrivate(PFrame frame, std::string reason) {
         frame->getPrivateData().free(mFramePrivateDataInstance, getMap().getGarbageCollector(), reason);
    }

    inline InitializationState getInitializationState() final {
        return mState;
    }

    void indirectMappingLoop();


    void updatePointDescriptor(PPoint mapPoint) {
        Optional<Binary256Descriptor> descriptor = findDescriptor(mapPoint);
        if (descriptor.has_value()) {
            mapPoint->setDescriptor(descriptor.value());
        }
    }

    void stop(std::string reason) override {
        AbstractSlam::stop(reason);
        mProcessQueue.destroy();
        mIndirectMappingQueue.destroy();
        mIndirectMappingQueue.destroy();
    }

private:
    const int INDIRECTKEYFRAME = getMap().createFrameGroup("Indirect Key Frame");
    const int DIRECTKEYFRAME = getMap().createFrameGroup("Direct Key Frame");

    const int IMMATUREINDIRECTPOINT = getMap().createMapPointGroup("Indirect immature");
    const int ACTIVEINDIRECTPOINT = getMap().createMapPointGroup("Indirect active");
    const int POINTSTOUPDATE = getMap().createMapPointGroup("Points to update");

    InitializationState mState;

    PrivateDataInstance mFramePrivateDataInstance;

    /// FUNCTIONS
    OptPFrame mLastDirectKeyFrame, mLastFrame;

    Optimization::DSOInitializer *mPhotometricInitializer;
    Optimization::DSOTracker *mPhotometricTracker;
    Optimization::DSOTracer *mPhotometricTracer;
    Optimization::DSOTracer *mHybridTracer;
    Optimization::DSOBundleAdjustment *mPhotometricBA;
    Optimization::DSOTrackerContext mLoopClosureTrackerContext;

    CornerAndDescriptor *mCornerExtractor;
    Features::BoWTracker *mInitTracker, *mMotionModelTracker, *mReferenceTracker, *mLocalPointsTracker, *mTriangulationTracker;
    Optimization::G2O::IndirectCameraOptimizer *mPnP;

    // Features::BoWTracker *mBoWTracker;
    Hartley2003Triangulation *mTriangulator;
    Robust::RobustRaulmurInitializer *mInitializer;
    Optimization::Ceres::IndirectBundleAdjustment *mIndirectCeresBundleAdjustment;
    Optimization::G2O::IndirectBundleAdjustment *mIndirectG2OBundleAdjustment;
    Optimization::IndirectPointOptimizer *mIndirectPointOptimizer;
    Features::Relocalization *mRelocalizer;

    Optimization::DSOTracker::Residual mLastPhotometricTrackingResidual;
    Optimization::G2O::IndirectCameraOptimizerResult mLastIndirectTrackingResult;
    float mFirstDirectRMSE = -1;
    bool mTrackingOk = false;
    bool mTrackedWithIndirect = false;
    bool mTrackedWithDirect = false;
    int mMatchesInliers = 0;

    bool mIndirectStopFlag;

    NN::Precomputed *mNeuralNetwork;

    // VIEWERS
    ReprojectionViewer *mReprojectionViewer;

    /// VARIABLES
    int mNumTrackingError = 0;
    int mDirectNeedKeyframeAfter = -1;
    bool mDirectNeedToKetchupMatching = false;

    // List<Matching> mMatchings;
    int mFirstTrackingMatchingNumber = -1;
    int mLastNumTrackedPoints = 0;
    bool mHaveValidDepthMap = false;
    Array2D<float> mDepthMap;
    BaMode mBaMode = BADIRECT; // BADIRECT for the initialization
    bool mShouldPreferDso = false;

    Mutex mFrameToFreeMutex;
    HashMap<PFrame, int, Hasher> mFrameToFree;

    Queue<OptPFrame, 100> mDirectMappingQueue;
   // Queue<OptPFrame, 100> mLoopClosingQueue;
    Queue<OptPFrame, 100> mIndirectMappingQueue;
    Set<PFrame, Hasher> mLocalKeyFrames;
    OptPFrame mReferenceKeyFrame;
    OptPFrame mLastRelocFrame;

    Window<Vector6> mTrackingDecisionCovariances, mBADecisionCovariances;
    Window<Vector2> mBADecisionScores;

    Queue<OptPFrame, 16> mProcessQueue;

    /// PARAMETERS
    bool mOnlyInitialize = false;
    int mFrameLimit = -1;
    Parameter mEnableNeuralNetwork = createParameter("enableNN", false);
    Parameter mEnableIndirect = createParameter("enableIndirect", true);
    Parameter mEnableDirect = createParameter("enableDirect", true);
    Parameter mLinearizeDirect = createParameter("linearizeDirect", true);
    Parameter mLinearizeIndirect = createParameter("linearizeIndirect", true);
    Parameter mEnableHybridPoint = createParameter("enableHybridPoint", false);
    Parameter mFreeAllDirectPoint = createParameter("freeAllDirectPoint", true);
    Parameter mNumOrbCorner = createParameter("numOrbCorner", 0);
    Parameter mBacondSaturatedRatio = createParameter("bacondSaturatedRatio", -1.0);
    Parameter mIndirectUncertaintyThreshold = createParameter("orbUncertaintyThreshold", -1.0);
    Parameter mScoreWeight = createParameter("bacondScoreWeight", -1.0);
    Parameter mScoreWindow = createParameter("bacondScoreWindow", 1);
    Parameter mTrackcondUncertaintyWeight = createParameter("trackcondUncertaintyWeight", -1.0);
    Parameter mTrackcondUncertaintyWindow = createParameter("trackcondUncertaintyWindow", 1);
    Parameter mBacondUncertaintyWeight = createParameter("bacondUncertaintyWeight", -1.0);
    Parameter mBacondUncertaintyWindow = createParameter("bacondUncertaintyWindow", 1);
    Parameter mBacondForce = createParameter("bacondForce", 0);
    Parameter mTrackcondForce = createParameter("trackcondForce", 0);
    Parameter mBaMinimumOrbPoint = createParameter("bacondMinimumOrbPoint", -1);
    Parameter mBaOrbRepeat = createParameter("baOrbRepeat", -1);

    Ptr<Statistic, NonNullable> mStatTrackORBVar = createStatistic("Track ORB Var");
    Ptr<Statistic, NonNullable> mStatTrackDSOVar = createStatistic("Track DSO Var");
    Ptr<Statistic, NonNullable> mStatBAORBNum = createStatistic("BA ORB Num");
    Ptr<Statistic, NonNullable> mStatBADSONum = createStatistic("BA DSO Num");



};

#endif //CML_ALL_HYBRID_H
