#include "Hybrid.h"


Hybrid::Hybrid() : AbstractSlam() {

    mFramePrivateDataInstance = getMap().getFramePrivataDataContext().createInstance();

    mPhotometricInitializer = new Optimization::DSOInitializer(this);
    mPhotometricInitializer->setAlias("dsoInitializer");
    mPhotometricTracker = new Optimization::DSOTracker(this);
    mPhotometricTracker->setViewableOnModel(true);
    mPhotometricTracker->setAlias("dsoTracker");
    mPhotometricTracer = new Optimization::DSOTracer(this);
    mPhotometricTracer->setAlias("dsoTracer");
    mHybridTracer = new Optimization::DSOTracer(this);
    mPhotometricBA = new Optimization::DSOBundleAdjustment(this);
    mPhotometricBA->setAlias("dsoBa");

    mReprojectionViewer = new ReprojectionViewer(this);
    mNeuralNetwork = new NN::Precomputed(".idepth.midas_large.png");
    //mReprojectionViewer->setViewable(true);

    if (mEnableIndirect.b()) {
        mCornerExtractor = new CornerAndDescriptor(this);
        mCornerExtractor->setAlias("orb");

        mInitTracker = new Features::BoWTracker(this, 0.9, true);
        mInitTracker->setAlias("initTracker");
        mMotionModelTracker = new Features::BoWTracker(this, 0.9, true);
        mMotionModelTracker->setAlias("motionModelTracker");
        mReferenceTracker = new Features::BoWTracker(this, 0.7, true);
        mReferenceTracker->setAlias("referenceTracker");
        mLocalPointsTracker = new Features::BoWTracker(this, 0.8, false);
        mLocalPointsTracker->setAlias("localPointsTracker");
        mTriangulationTracker = new Features::BoWTracker(this, 0.6, false);
        mTriangulationTracker->setAlias("triangulationTracker");

        mTriangulator = new Hartley2003Triangulation(this);
        mInitializer = new Robust::RobustRaulmurInitializer(this);
        mIndirectCeresBundleAdjustment = new Optimization::Ceres::IndirectBundleAdjustment(this);
        mIndirectG2OBundleAdjustment = new Optimization::G2O::IndirectBundleAdjustment(this);
        mIndirectG2OBundleAdjustment->setAlias("orbBa");
        mIndirectPointOptimizer = new Optimization::IndirectPointOptimizer(this);
        mPnP = new Optimization::G2O::IndirectCameraOptimizer(this);
    }

#if CML_HAVE_LIBZIP
    mCornerExtractor->loadVocabulary("resources/ORBvoc.zip");
#else
    mCornerExtractor->loadVocabulary("resources/ORBvoc.txt");
#endif
}

Hybrid::~Hybrid() {
    //  delete mReprojectionTracker;
    delete mIndirectCeresBundleAdjustment;
    delete mInitializer;
    delete mTriangulator;
    delete mCornerExtractor;
    delete mReprojectionViewer;
    delete mPhotometricBA;
    delete mPhotometricTracer;
    delete mPhotometricTracker;
    delete mPhotometricInitializer;
}


void Hybrid::onReset() {
    CML::logger.info("Resetting...");
    mState = NOT_INITIALIZED;
    mReferenceKeyFrame = nullptr;
    mLastDirectKeyFrame = nullptr;
    mLastFrame = nullptr;
    mLastPhotometricTrackingResidual = Optimization::DSOTracker::Residual();
    mFirstDirectRMSE = -1;
    mTrackingOk = false;
    mTrackedWithIndirect = false;
    mTrackedWithDirect = false;
    mMatchesInliers = 0;
    mNumTrackingError = 0;
    mDirectNeedKeyframeAfter = -1;
    mDirectNeedToKetchupMatching = false;
    mFirstTrackingMatchingNumber = -1;
    mBaMode = BADIRECT;
    mFrameToFree = HashMap<PFrame, int>();
    mLocalKeyFrames = Set<PFrame>();
    mLastRelocFrame = nullptr;
}

void Hybrid::run() {

    assertDeterministicMsg("Run");


    mPhotometricBA->setMixedBundleAdjustment(mMixedBundleAdjustment.b());

    if (mEnableIndirect.b()) {
        mCornerExtractor->setNumFeatures(mNumOrbCorner.i());
        mHybridTracer->setPointDensity(mNumOrbCorner.i() / 8);
    }

    CML::logger.info("Running...");
    Ptr<SlamThread, Nullable> directMappingThread;
    if (mEnableDirect.b() && !mLinearizeDirect.b()) {
        directMappingThread = new SlamThread(*this, "Direct Mapping", [this] { directMappingLoop(); });
    }
    Ptr<SlamThread, Nullable> indirectMappingThread;
    if (mEnableIndirect.b() && !mLinearizeIndirect.b()) {
        indirectMappingThread = new SlamThread(*this, "Indirect Mapping", [this] { indirectMappingLoop(); });
    }

    OptPFrame currentFrame;


   // std::thread *processThread = new std::thread(&Hybrid::processThread, this);

    beforeStart();

    while (!isStopped()) {

        currentFrame = getNextFrame();
        if (currentFrame.isNull()) {
            stop("This is the end of the video");
            break;
        }
        onNewFrame(currentFrame);

        if (mFrameLimit >= 0 && (int) currentFrame->getId() >= mFrameLimit) {
            stop("Frame limit reached");
            break;
        }

        extractOrb(currentFrame);

        //*mProcessQueue.getPushElement() = currentFrame;
        //mProcessQueue.notifyPush();
        processFrame(currentFrame);

        if (mTrackingOk) {
            onTracked(currentFrame);
        }
    }



    CML::logger.info("End of SLAM");

}

void Hybrid::processThread() {
    while (!isStopped()) {
        OptPFrame frame = *mProcessQueue.getPopElement();
        mProcessQueue.notifyPop();

        if (frame.isNull()) {
            return;
        }

        processFrame(frame);
    }
}

void Hybrid::processFrame(PFrame currentFrame) {
    addFrame(currentFrame);

    if (mState == NOT_INITIALIZED) {
        initializeWithDSO(currentFrame);
        // initializeWithRaulmur(currentFrame);
        if (mOnlyInitialize && mState != NOT_INITIALIZED) {
            stop("We only want to initialize");
            return;
        }
    } else {

        if (!mEnableDirect.b()) {
            mTrackingOk = false;
            trackWithOrbAndDsoRefinement(currentFrame);
        } else if (!mEnableIndirect.b()) {
            mTrackingOk = false;
            trackWithDso(currentFrame);
        } else {

            mShouldPreferDso = poseEstimationDecision();
            // assertThrow(mShouldPreferDso, "Should prefer dso");
            if (mShouldPreferDso) {
                logger.info("Should Prefer Dso");
                mBacondTrack.add(0);
                mTrackingOk = false;
                mStatTrackDec->addValue(0);
                trackWithDso(currentFrame);
            } else {
                logger.info("Using Orb with Dso Refinement");
                mBacondTrack.add(1);
                mTrackingOk = false;
                mStatTrackDec->addValue(1);
                trackWithOrbAndDsoRefinement(currentFrame);
            }

        }

        if (!mFixedReferenceFrame) {
            if (!mTrackingOk) {
                mNumTrackingError++;
                if (mNumTrackingError > 3) {
                    restartOrStop("Too many tracking error");
                }
            } else {
                mNumTrackingError = 0;
            }
        }


        if (isStopped()) {
            return;
        }

        if (mTrackingOk && !mFixedReferenceFrame) {

            if (mMixedBundleAdjustment.b()) {

                bool needKF = indirectNeedNewKeyFrame(currentFrame) || directNeedNewKeyFrame(currentFrame);
                mBaMode = BADIRECT;
                indirectPostprocess(currentFrame, needKF);
                directPostprocess(currentFrame, needKF);

            } else {

                bool needIndirectKF = mEnableIndirect.b() && indirectNeedNewKeyFrame(currentFrame);
                bool needDirectKF = mEnableDirect.b() && directNeedNewKeyFrame(currentFrame);

                if (needIndirectKF || needDirectKF) {

                    mBaMode = bundleAdjustmentDecision(needIndirectKF, needDirectKF);

                    if (mBaMode == BADIRECT) {
                        mStatBADec->addValue(0);
                        logger.important("Ba mode Direct");
                        directPostprocess(currentFrame, needDirectKF);
                        indirectPostprocess(currentFrame, needIndirectKF);
                    } else {
                        mStatBADec->addValue(1);
                        logger.important("Ba mode Indirect");
                        indirectPostprocess(currentFrame, needIndirectKF);
                        directPostprocess(currentFrame, needDirectKF);
                    }

                }

            }

            mLastFrame = currentFrame;

        } else {

            logger.error("Tracking is not okay");
            freePrivate(currentFrame, "Hybrid::run (tracking is not okay)"); // If the tracking is not ok, do nothing with this frame, consider it as outlier

        }

        /*bool isKeyFrame = currentFrame->isGroup(INDIRECTKEYFRAME) || currentFrame->isGroup(DIRECTKEYFRAME) || currentFrame->isGroup(getMap().KEYFRAME);
        if (isKeyFrame) {

        } else {

        }*/

    }
}

void Hybrid::indirectPostprocess(PFrame currentFrame, bool needKF) {
    if (!mEnableIndirect.b()) {
        return;
    }
    logger.debug("Indirect postprocess");
    if (needKF || (mBaMode != BAINDIRECT && mLastNumTrackedPoints < 15)) {
        //currentFrame->setGroup(getMap().KEYFRAME, true);
        if (mLinearizeIndirect.b()) {
            indirectMap(currentFrame);
        } else {
            *mIndirectMappingQueue.getPushElement() = currentFrame;
            mIndirectMappingQueue.notifyPush();
            mIndirectStopFlag = true;
        }
    } else {
        LockGuard lg(mFrameToFreeMutex);
        mFrameToFree[currentFrame] = 0;
    }
}

void Hybrid::directPostprocess(PFrame currentFrame, bool needKF) {
    if (!mEnableDirect.b()) {
        return;
    }
    logger.debug("Direct postprocess");
    if (needKF) {
        if (mLinearizeDirect.b()) {
            directMap(currentFrame);
        } else {
            mDirectNeedKeyframeAfter = currentFrame->getId();
        }
    } else {
        if (mLinearizeDirect.b()) {
            directMakeNonKeyFrame(currentFrame);
        }
    }

    if (!mLinearizeDirect.b()) {
        *mDirectMappingQueue.getPushElement() = currentFrame;
        mDirectMappingQueue.notifyPush();
    }

}

void Hybrid::trackWithOrbAndDsoRefinement(PFrame currentFrame) {
    mTrackedWithIndirect = false;
    mTrackedWithDirect = false;
    /*if (mEnableHybridTracking && mLastMappingFrame.isNotNull()) {
        hybridLocalization(currentFrame, mLastMappingFrame, false, mEnableIndirect);
    }*/
    scalar_t mode = 0;
    scalar_t modeSum = 0;

    if (mEnableIndirect.b() && !mTrackingOk) {
        mTrackingOk = indirectTrackWithMotionModel(currentFrame);
        if (mTrackingOk) {
            mode = mode + 2;
            modeSum = modeSum + 2;
            mTrackedWithIndirect = true;
        }
    }

    if (mEnableIndirect.b() && !mTrackingOk) {
        mTrackingOk = indirectTrackReferenceKeyFrame(currentFrame);
        if (mTrackingOk) {
            mode = mode + 2;
            modeSum = modeSum + 2;
            mTrackedWithIndirect = true;
        }
    }

    if (!mTrackingOk) {
        logger.error("ORB Tracking failed");
    }

    /* if (!mTrackingOk) {
         pauseHere();

     }*/

    if (mEnableDirect.b()) {

        if (mTrackingOk) {
            currentFrame->setGroup(ORBTRACKEDFRAME, true);
            Camera camera = currentFrame->getCamera();
            Exposure exposure = currentFrame->getExposure();
            mLastPhotometricTrackingResidual = mPhotometricTracker->optimize(mLastPhotometricTrackingResidual, 0, currentFrame, mPhotometricTracker->getLastComputed(), camera, exposure);
            if (mLastPhotometricTrackingResidual.isCorrect && mLastPhotometricTrackingResidual.saturatedRatio() < 0.15) { // todo : treshold here and in other file
                currentFrame->setCamera(camera);
                logger.info("Refined with DSO");
                mode = mode + 0;
                modeSum = modeSum + 1;
            } else {
                if (mCheckPoseEstimationWithDso.b()) {
                    mTrackingOk = false;
                }
            }
            if (mLastPhotometricTrackingResidual.isCorrect) {
                mPhotometricTracker->addStatistic(mLastPhotometricTrackingResidual, exposure);
                mode = mode + 0;
                modeSum = modeSum + 1;
                mTrackedWithDirect = true;
                currentFrame->setExposureParameters(exposure);
            }
        }

        if (!mTrackingOk) {
            currentFrame->setGroup(RECOVEREDFRAME, true);
            mTrackingOk = mPhotometricTracker->trackWithMotionModel(currentFrame, mPhotometricTracker->getLastComputed(), getMap().getLastFrame(1)->getCamera(),getMap().getLastFrame(2)->getCamera(), mLastPhotometricTrackingResidual);
            if (mTrackingOk) {
                mTrackedWithDirect = true;
                mode = mode + 0;
                modeSum = modeSum + 2;
            }
        }
    }

    if (mEnableIndirect.b() && mTrackingOk) {
        if (!mTrackedWithIndirect) {
            mLastRelocFrame = currentFrame;
        }
        mTrackedWithIndirect = indirectTrackLocalMap(currentFrame);
        if (mTrackedWithIndirect) {
            mode = mode + 1;
            modeSum = modeSum + 1;
        } else {
            /*  if (trackedOneTimeWithIndirect) {
                  pauseHere();
              }*/
        }
    }

}

void Hybrid::trackWithDso(PFrame currentFrame) {
    assertThrow(mEnableDirect.b(), "You can't call this function with direct disabled");

    currentFrame->setGroup(DSOTRACKEDFRAME, true);

    mTrackedWithIndirect = false;
    mTrackedWithDirect = false;

    scalar_t mode = 0;
    scalar_t modeSum = 0;

    mTrackingOk = mPhotometricTracker->trackWithMotionModel(currentFrame, mPhotometricTracker->getLastComputed(), getMap().getLastFrame(1)->getCamera(),getMap().getLastFrame(2)->getCamera(), mLastPhotometricTrackingResidual);
    if (mTrackingOk) {
        mTrackedWithDirect = true;
        mode = mode + 0;
        modeSum = modeSum + 2;
    }

    if (mEnableIndirect.b() && mTrackingOk) {
        mTrackedWithIndirect = indirectTrackLocalMap(currentFrame);
        if (mTrackedWithIndirect) {
            mode = mode + 1;
            modeSum = modeSum + 1;
        }
    }


}

void Hybrid::initializeWithDSO(PFrame currentFrame) {
    if (mLastDirectKeyFrame.isNull()) {
        mLastDirectKeyFrame = currentFrame;
        mLastDirectKeyFrame->setGroup(getMap().KEYFRAME, true);
        mLastDirectKeyFrame->setGroup(DIRECTKEYFRAME, true); // Need to do this to save the capture frame
        if (mEnableNeuralNetwork.b()) {
            try {
                mDepthMap = mNeuralNetwork->load(currentFrame->getCaptureFrame()).cast<float>(); // Put the points between 0 and 2
                mHaveValidDepthMap = true;
                logger.important("Successfully loaded neural network depth map for first frame");
            } catch (...) {
                mHaveValidDepthMap = false;
                logger.error("Can't load neural network depth map for first frame");
                abort();
            }
        }
        return;
    }

    int result;
    if (mHaveValidDepthMap) {
        result = mPhotometricInitializer->tryInitialize(currentFrame, mLastDirectKeyFrame, mDepthMap);
    } else {
        result = mPhotometricInitializer->tryInitialize(currentFrame, mLastDirectKeyFrame);
    }

    if (result == -1) {
        mLastDirectKeyFrame = nullptr;
    }

    if (result == 1) {
        assertThrow(mLastDirectKeyFrame != currentFrame, "Some strange bug is happening here");

        PFrame lastDirectKeyFrame = mLastDirectKeyFrame;

        mLastDirectKeyFrame->setGroup(getMap().KEYFRAME, true);
        currentFrame->setGroup(getMap().KEYFRAME, true);
        mLastDirectKeyFrame->setGroup(INDIRECTKEYFRAME, true);
        currentFrame->setGroup(INDIRECTKEYFRAME, true);
        mLastDirectKeyFrame->setGroup(DIRECTKEYFRAME, true);
        currentFrame->setGroup(DIRECTKEYFRAME, true);

        if (!mOnlyInitialize) {

            mPhotometricBA->addNewFrame(mLastDirectKeyFrame, mPhotometricTracer->IMMATUREPOINT);
            mPhotometricBA->addPoints(getMap().getMapPoints());

            if (mEnableDirect.b()) {
                directMap(currentFrame, true);
            }

            if (mEnableIndirect.b()) {
                // indirectCreateNewImmaturePoint(currentFrame);
                needVocabularyFor(lastDirectKeyFrame);
                needVocabularyFor(currentFrame);
                auto lastDirectKeyFrameData = get(lastDirectKeyFrame);
                auto mCurrentFrameData = get(currentFrame);
                List<Matching> matchings = mTriangulationTracker->trackForTriangulation(
                        {lastDirectKeyFrame, lastDirectKeyFrameData->featureId, lastDirectKeyFrameData->descriptors},
                        {currentFrame, mCurrentFrameData->featureId, mCurrentFrameData->descriptors},
                        2);

                auto currentNewImmaturePoints = indirectCreateNewImmaturePointFromMatchings(matchings);

            }

        }

        mReferenceKeyFrame = lastDirectKeyFrame;
        mLastFrame = currentFrame;

        mState = OK;
    } else {

        freePrivate(currentFrame, "Hybrid::initializeWithDSO (the frame is not a keyframe)");

    }

}






