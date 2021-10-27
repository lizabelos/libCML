#include "Hybrid.h"

void Hybrid::directMappingLoop() {

    OptPFrame frame = *mDirectMappingQueue.getPopElement();
    if (frame.isNull()) {
        return;
    }
    mDirectMappingQueue.notifyPop();

    if(mDirectMappingQueue.getCurrentSize() > 3) {
        mDirectNeedToKetchupMatching = true;
    }

    if(mDirectMappingQueue.getCurrentSize() > 0) // if there are other frames to tracke, do that first.
    {
        directMakeNonKeyFrame(frame);

        if(mDirectNeedToKetchupMatching && mDirectMappingQueue.getCurrentSize() > 0)
        {
            frame = *mDirectMappingQueue.getPopElement();
            mDirectMappingQueue.notifyPop();

            //fh->shell->camToWorld = fh->shell->trackingRef->camToWorld * fh->shell->camToTrackingRef;
            //fh->setEvalPT_scaled(fh->shell->camToWorld.inverse(),fh->shell->aff_g2l);
        }

    }
    else
    {
        if(mDirectNeedKeyframeAfter >= (int)mLastDirectKeyFrame->getId())
        {
            directMap(frame);
            mDirectNeedToKetchupMatching = false;
        }
        else
        {
            directMakeNonKeyFrame(frame);
        }
    }
}

void Hybrid::directMakeNonKeyFrame(PFrame currentFrame) {
    mPhotometricTracer->traceNewCoarse(currentFrame, mPhotometricBA->ACTIVEKEYFRAME);
}

void Hybrid::directMap(PFrame currentFrame, bool callFromInitialization) {
    logger.info("Mapping of frame : " + std::to_string(currentFrame->getId()));

    auto currentFrameData = get(currentFrame);

    currentFrame->setGroup(DIRECTKEYFRAME, true);
    currentFrame->setGroup(getMap().KEYFRAME, true);


    //PFrame lastLastKeyFrame = mLastDirectKeyFrame;

    Timer timer;

    mPhotometricTracer->traceNewCoarse(currentFrame, mPhotometricBA->ACTIVEKEYFRAME);
    mPhotometricBA->addNewFrame(currentFrame, mPhotometricTracer->IMMATUREPOINT);
    Set<PPoint, Hasher> photometricPoints = mPhotometricTracer->activatePoints(mPhotometricBA->ACTIVEKEYFRAME, mPhotometricBA->ACTIVEPOINT);
    mPhotometricBA->addPoints(photometricPoints);

    if (mEnableHybridPoint.b()) {
        mHybridTracer->traceNewCoarse(currentFrame, INDIRECTKEYFRAME);
        Set<PPoint, Hasher> hybridPoints = mHybridTracer->activatePoints(INDIRECTKEYFRAME, ACTIVEINDIRECTPOINT);
        for (auto point : hybridPoints) {
            updatePointDescriptor(point);
            point->setGroup(ACTIVEINDIRECTPOINT, true);
            //point->setGroup(IMMATUREINDIRECTPOINT, true);
            point->setGroup(getMap().INDIRECTGROUP, true);
            point->setGroup(getMap().DIRECTGROUP, false);
        }
        mHybridTracer->makeNewTracesFrom(currentFrame, currentFrameData->featureId);

        if (mIndirectMappingQueue.getCurrentSize() > 1) {
            timer.stop();
            mIndirectMappingQueue.notifyPop();
            return;
        }
    }

    timer.start();
    bool ok = mPhotometricBA->run(mBaMode != BADIRECT);
    timer.stopAndPrint("Photometric BA run");

    if (mBaMode == BADIRECT && !ok) {
        logger.error("BA failed.");
        restartOrStop("BA failed");
        return;
    }

    // Do this the first, this is the most important for the tracking
    timer.start();
    mPhotometricTracker->makeCoarseDepthL0(currentFrame, mPhotometricBA->getGoodPointsForTracking());
    timer.stopAndPrint("Make coarse depth l0");

    // =========================== REMOVE OUTLIER =========================
    // Directly done in the function removeResiduals


    // mPhotometricTracker->resetFirstRMSE();

    // =========================== (Activate-)Marginalize Points =========================
    timer.start();
    mPhotometricBA->tryMarginalize();
    for (auto outlier : mPhotometricBA->getOutliers()) {
        mPhotometricBA->removePoint(outlier);
        getMap().removeMapPoint(outlier);
    }
    mPhotometricBA->computeNullspaces();
    mPhotometricBA->marginalizePointsF();
    //  pauseHere();

    mPhotometricTracer->makeNewTraces(currentFrame);

    List<PFrame> marginalized = mPhotometricBA->marginalizeFrames();

    OptPFrame oldestActiveKeyframe;
    for (auto frame : getMap().getGroupFrames(mPhotometricBA->ACTIVEKEYFRAME)) {
        if (oldestActiveKeyframe.isNull() || frame->getId() < oldestActiveKeyframe->getId()) {
            oldestActiveKeyframe = frame;
        }
    }
    for (auto frame : marginalized) {

        for (auto point : frame->getReferenceGroupMapPoints(getMap().DIRECTGROUP)) {

            scalar_t d = 1.0 / point->getReferenceInverseDepth();
            scalar_t c = point->getUncertainty();

            if (d * d * d * d * c > 0.00001 || mFreeAllDirectPoint.b()) {
                getMap().removeMapPoint(point);
            }

        }

       // if (frame->getId() > oldestActiveKeyframe->getId()) {
            mPhotometricTracker->free(frame, "Hybrid::directMap (marginalized frame)");
            frame->getCaptureFrame().makeUnactive(); // todo : thread safety bug
            // frameToAddToBA.emplace_back(frame);
       // }

    }

    if (mEnableIndirect.b() && (*getMap().getGroupFrames(mPhotometricBA->ACTIVEKEYFRAME).rbegin())->getId() > 1) {
        PFrame lastActiveKeyFrame = (*getMap().getGroupFrames(mPhotometricBA->ACTIVEKEYFRAME).rbegin());
        if (!lastActiveKeyFrame->isGroup(mRelocalizer->LOOPCLOSUREFRAMEGROUP)) {
            lastActiveKeyFrame->setGroup(mRelocalizer->LOOPCLOSUREFRAMEGROUP, true);
        }
    }

    mLastDirectKeyFrame = currentFrame;
    mFirstDirectRMSE = -1;
    mFirstTrackingMatchingNumber = -1;

    timer.stopAndPrint("Marginalization");
}

