//
// Created by thomas on 22/02/2021.
//

#ifndef CML_HYBRID_H
#define CML_HYBRID_H

#include <thread>

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
//#include <cml/features/corner/OpenCV.h>

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


        for (auto frame : getMap().getGroupFrames(ORBTRACKEDFRAME)) {
            drawBoard.color(0, 1, 0);
            drawBoard.lineWidth(5);
            drawBoard.paintCamera(frame->getCamera());
        }

        for (auto frame : getMap().getGroupFrames(DSOTRACKEDFRAME)) {
            drawBoard.color(0, 0, 1);
            drawBoard.lineWidth(5);
            drawBoard.paintCamera(frame->getCamera());
        }

        for (auto frame : getMap().getGroupFrames(RECOVEREDFRAME)) {
            drawBoard.color(1, 0, 0);
            drawBoard.lineWidth(5);
            drawBoard.paintCamera(frame->getCamera());
        }

        /*for (auto point : getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT)) {
            drawBoard.color(0, 0, 1);
            drawBoard.pointSize(2);
            drawBoard.point((Vector3f)point->getWorldCoordinate().absolute().cast<float>());
        }*/

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
    virtual void beforeStart() {

    }

    virtual void onNewFrame(PFrame currentFrame) {

    }

    void setFlatFixedReferenceFrame(PFrame frame, int &directFid) {
        mReferenceKeyFrame = frame;
        mLastFrame = frame;

        extractOrb(mReferenceKeyFrame);

        auto referenceFrameData = get(mReferenceKeyFrame);
        for (int i = 0; i < mReferenceKeyFrame->getFeaturePoints(referenceFrameData->featureId).size(); i++) {
            auto mapPoint = this->getMap().createMapPoint(mReferenceKeyFrame, FeatureIndex(referenceFrameData->featureId, i), MapPointType::INDIRECTTYPE);
            mReferenceKeyFrame->setMapPoint(FeatureIndex(referenceFrameData->featureId, i), mapPoint);
            mapPoint->setReferenceInverseDepth(1);
            mapPoint->setGroup(this->getMap().MAPPED, true);
            mapPoint->setDescriptor(referenceFrameData->descriptors[i]);
        }

        List<Corner> directCorners;
        List<float> directTypes;

        if (mPixelSelector == nullptr) {
            mPixelSelector = new Features::PixelSelector(this, frame->getWidth(0), frame->getHeight(0));
        }
        mPixelSelector->compute(mReferenceKeyFrame->getCaptureFrame(), directCorners, directTypes, 2000);

        directFid = mReferenceKeyFrame->addFeaturePoints(directCorners);
        PointSet directPoints;
        for (int i = 0; i < mReferenceKeyFrame->getFeaturePoints(directFid).size(); i++) {
            auto mapPoint = this->getMap().createMapPoint(mReferenceKeyFrame, FeatureIndex(directFid, i), MapPointType::DIRECTTYPE);
            mReferenceKeyFrame->setMapPoint(FeatureIndex(directFid, i), mapPoint);
            mapPoint->setReferenceInverseDepth(1);
            mapPoint->setGroup(this->getMap().MAPPED, true);
            directPoints.insert(mapPoint);
        }
        mPhotometricTracker->makeCoarseDepthL0(mReferenceKeyFrame, directPoints);

        mFixedReferenceFrame = true;

        mState = InitializationState::OK;

    }

    virtual void onTracked(PFrame frame) {

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
    bool indirectTrackWithCMLGraph(PFrame currentFrame);

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
    const int ORBTRACKEDFRAME = getMap().createFrameGroup("ORB Tracked Frame");
    const int DSOTRACKEDFRAME = getMap().createFrameGroup("DSO Tracked Frame");
    const int RECOVEREDFRAME = getMap().createFrameGroup("Recovered Frame");

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
    Features::PixelSelector *mPixelSelector = nullptr;

    CornerAndDescriptor *mCornerExtractor;
    Features::BoWTracker *mInitTracker, *mMotionModelTracker, *mReferenceTracker, *mLocalPointsTracker, *mTriangulationTracker;
    Optimization::G2O::IndirectCameraOptimizer *mPnP;

    // Features::BoWTracker *mBoWTracker;
    Hartley2003Triangulation *mTriangulator;
    Robust::RobustRaulmurInitializer *mInitializer;
    Optimization::Ceres::IndirectBundleAdjustment *mIndirectCeresBundleAdjustment;
    Optimization::G2O::IndirectBundleAdjustment *mIndirectG2OBundleAdjustment;
    Optimization::IndirectPointOptimizer *mIndirectPointOptimizer;

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

    float mLastOrbTrackingInliersRatio = 0;
    int mLastHaveSucceedCVMM = 0;

    Mutex mFrameToFreeMutex;
    FrameHashMap<int> mFrameToFree;

    Queue<OptPFrame, 100> mDirectMappingQueue;
   // Queue<OptPFrame, 100> mLoopClosingQueue;
    Queue<OptPFrame, 100> mIndirectMappingQueue;
    FrameSet mLocalKeyFrames;
    OptPFrame mReferenceKeyFrame;
    OptPFrame mLastRelocFrame;

    bool mFixedReferenceFrame = false;

    Window<Vector6> mTrackingDecisionCovariances, mBADecisionCovariances;
    Window<Vector2> mBADecisionScores;

    Queue<OptPFrame, 16> mProcessQueue;

    /// PARAMETERS
    bool mOnlyInitialize = false;
    int mFrameLimit = -1;
    Parameter mEnableIndirect = createParameter("enableIndirect", true);
    Parameter mEnableDirect = createParameter("enableDirect", true);
    Parameter mLinearizeDirect = createParameter("linearizeDirect", true);
    Parameter mLinearizeIndirect = createParameter("linearizeIndirect", true);
    Parameter mFreeAllDirectPoint = createParameter("freeAllDirectPoint", false);
    Parameter mNumOrbCorner = createParameter("numOrbCorner", 500);

    Parameter mBacondSaturatedRatio = createParameter("bacondSaturatedRatio", 0.4);
    Parameter mBacondSaturatedRatioDir = createParameter("bacondSaturatedRatioDir", true);
    Parameter mIndirectUncertaintyThreshold = createParameter("orbUncertaintyThreshold", -1.0);
    Parameter mScoreWeight = createParameter("bacondScoreWeight", 0.0);
    Parameter mScoreWindow = createParameter("bacondScoreWindow", 1);
    Parameter mTrackcondUncertaintyWeight = createParameter("trackcondUncertaintyWeight", 0.65);
    Parameter mTrackcondUncertaintyWeightOrb = createParameter("trackcondUncertaintyWeightOrb", -1.0f);
    Parameter mTrackcondUncertaintyWeightDso = createParameter("trackcondUncertaintyWeightDso", -1.0f);
    Parameter mTrackcondUncertaintyWindow = createParameter("trackcondUncertaintyWindow", 1);
    Parameter mBacondUncertaintyWeight = createParameter("bacondUncertaintyWeight", -1.0);
    Parameter mBacondUncertaintyWindow = createParameter("bacondUncertaintyWindow", 1);
    Parameter mBacondForce = createParameter("bacondForce", 0);
    Parameter mTrackcondForce = createParameter("trackcondForce", 0);
    Parameter mBaMinimumOrbPoint = createParameter("bacondMinimumOrbPoint", 40);
    Parameter mTrackingMinimumOrbPoint = createParameter("trackingMinimumOrbPoint", 85);
    Parameter mBaOrbRepeat = createParameter("baOrbRepeat", -1);
    Parameter mOrbInlierRatioThreshold = createParameter("orbInlierRatioThreshold", 0.6f);
    Parameter mOrbInlierNumThreshold = createParameter("orbInlierNumThreshold", 30);
    Parameter mBacondTrackThresholdOrb = createParameter("bacondTrackThresholdOrb", -1.0);
    Parameter mBacondTrackThresholdDso = createParameter("bacondTrackThresholdDso", -1.0);
    Parameter mTrackcondFlowThreshold = createParameter("trackcondFlowThreshold", -1.0);
    Window<float> mBacondTrack;


    Parameter mOptimiseOrbEachTime = createParameter("mOptimiseOrbEachTime", false);

    Parameter mOrbKeyframeReflimit = createParameter("orbKeyframeReflimit", 200);
    Parameter mOrbKeyframeRatio = createParameter("orbKeyframeRatio", 0.94f);

    Parameter mDsoKeyframeWeight = createParameter("dsoKeyframeWeight", 1.0f);
    Parameter mDsoKeyframeResidualRatio = createParameter("dsoKeyframeResidualRatio", 2.0f);

    Parameter mMixedBundleAdjustment = createParameter("mixedBundleAdjustment", false);

    Parameter mDsoKeyframeSkipOnFailure = createParameter("dsoKeyframeSkipOnFailure", false);
    Parameter mDsoKeyframeSkipOnNoTrack = createParameter("dsoKeyframeSkipOnNoTrack", false);
    Parameter mOrbKeyframeSkipOnFailure = createParameter("orbKeyframeSkipOnFailure", false);
    Parameter mOrbKeyframeMinimumPoints = createParameter("orbKeyframeMinimumPoints", -1);

    Parameter mCheckPoseEstimationWithDso = createParameter("checkPoseEstimationWithDso", false);
    Parameter mDsoUrgentlyNeedNewPointsKFRatio = createParameter("dsoUrgentlyNeedNewPointsKFRatio", 1.0f);

    Parameter mNumOrbMultiplier = createParameter("numOrbMultiplier", 1.0f);
    Parameter mEnableIndirectCulling = createParameter("enableIndirectCulling", true);

    Parameter mGraphTrackDist = createParameter("graphTrackDist", 14);
    Parameter mGraphDistScore = createParameter("graphDistScore", 1000);
    Parameter mGraphDescriptorScoreA = createParameter("graphDescriptorScoreA", 1000);
    Parameter mGraphDescriptorScoreB = createParameter("graphDescriptorScoreB", 1000);
    Parameter mGraphLevelScore = createParameter("graphLevelScore", 1000);
    Parameter mGraphScoreThreshold = createParameter("graphScoreThreshold", 2000);
    Parameter mGraphScoreRatio = createParameter("graphScoreRatio", 0.8f);
    Parameter mGraphMininumMatching = createParameter("graphMininumMatching", 50);



    Ptr<Statistic, NonNullable> mStatTrackORBVar = createStatistic("Track ORB Var");
    Ptr<Statistic, NonNullable> mStatTrackDSOVar = createStatistic("Track DSO Var");
    Ptr<Statistic, NonNullable> mStatBAORBNum = createStatistic("BA ORB Num");
    Ptr<Statistic, NonNullable> mStatBADSONum = createStatistic("BA DSO Num");
    Ptr<Statistic, NonNullable> mStatTrackDec = createStatistic("Tracking Decision");
    Ptr<Statistic, NonNullable> mStatBADec = createStatistic("Bundle Adjustment Decision");

    Ptr<StatisticTimer, NonNullable> mIndirectPETimer = createStatisticTimer("Indirect PE Timer");
    Ptr<StatisticTimer, NonNullable> mDirectPETimer = createStatisticTimer("Direct PE Timer");
    Ptr<StatisticTimer, NonNullable> mIndirectBATimer = createStatisticTimer("Indirect BA Timer");
    Ptr<StatisticTimer, NonNullable> mDirectBATimer = createStatisticTimer("Direct BA Timer");
    Ptr<StatisticTimer, NonNullable> mTotalTimer = createStatisticTimer("Total Timer");

};

#endif //CML_ALL_HYBRID_H
