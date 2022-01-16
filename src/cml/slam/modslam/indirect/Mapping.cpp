#include <cml/config.h>
#include "Hybrid.h"


void Hybrid::indirectMappingLoop() {

    OptPFrame currentFrame = *mIndirectMappingQueue.getPopElement();

    if (currentFrame.isNull()) {
        return;
    }

    indirectMap(currentFrame);
    mIndirectMappingQueue.notifyPop();


}

void Hybrid::indirectMap(PFrame currentFrame) {
    Timer timer;
    timer.start();

    //auto currentFrameData = get(currentFrame);

    if (mIndirectMappingQueue.getCurrentSize() > 1) {
        logger.info("Stopping indirect map because one frame is in the queue");
        timer.stop();
        mIndirectMappingQueue.notifyPop();
        return;
    }

    List<PFrame> toFree;
    {
        LockGuard lg(mFrameToFreeMutex);
        for (auto &[frame, count] : mFrameToFree) {
            count++;
            if (count >= 7) {
                toFree.emplace_back(frame);
            }
        }
        for (auto frame : toFree) {
            mFrameToFree.erase(frame);
        }
    }
    for (auto frame : toFree) {
        freePrivate(frame, "This frame is not a keyframe");
    }

    currentFrame->setGroup(INDIRECTKEYFRAME, true);

    indirectTrackImmature(currentFrame);
    List<PPoint> newImmaturePoints = indirectCreateNewImmaturePoint(currentFrame);

    if (mIndirectMappingQueue.getCurrentSize() > 1) {
        logger.info("Stopping indirect map because one frame is in the queue");
        timer.stop();
        mIndirectMappingQueue.notifyPop();
        return;
    }

    indirectSearchInNeighbors(currentFrame);

    if (mIndirectMappingQueue.getCurrentSize() > 1) {
        logger.info("Stopping indirect map because one frame is in the queue");
        timer.stop();
        mIndirectMappingQueue.notifyPop();
        return;
    }

    indirectLocalOptimize(currentFrame);
    if (mIndirectMappingQueue.getCurrentSize() > 1) {
        logger.info("Stopping indirect map because one frame is in the queue");
        timer.stop();
        mIndirectMappingQueue.notifyPop();
        return;
    }

    keyframeCulling();

    timer.stop();
}

void Hybrid::indirectLocalOptimize(PFrame currentFrame) {
    if (mBaMode == BAINDIRECT) {
        mIndirectStopFlag = false;
        bool result = mIndirectG2OBundleAdjustment->localOptimize(currentFrame, INDIRECTKEYFRAME, &mIndirectStopFlag);
        if (result) {
            mIndirectG2OBundleAdjustment->apply();
        }
    }
    else {
     /*   mIndirectStopFlag = false;
        bool result = mIndirectG2OBundleAdjustment->localOptimize(currentFrame, INDIRECTKEYFRAME, &mIndirectStopFlag, true);
        if (result) {
            mIndirectG2OBundleAdjustment->apply();
        }
       */ List<PPoint> points = getMap().getGroupMapPointsAsList(ACTIVEINDIRECTPOINT);

      /*  for (size_t i = 0; i < points.size(); i++) {
            List<PFrame> frames;
            frames.reserve(50);
            for (auto frame : points[i]->getIndirectApparitions()) {
                if (frame->isGroup(INDIRECTKEYFRAME)) {
                    frames.emplace_back(frame);
                }
            }
            mIndirectCeresBundleAdjustment->optimizeSinglePoint(points[i], frames, false);
        } */

    }
}

void Hybrid::keyframeCulling() {

    int numKeyframes = 3;
    int numframes = 0;
    int centerframe = 1;
    OptPFrame keyframes[numKeyframes];
    for (auto frame : getMap().getGroupFrames(INDIRECTKEYFRAME)) {

        for (int i = 0; i < numKeyframes - 1; i++) {
            keyframes[i] = keyframes[i + 1];
        }
        keyframes[numKeyframes - 1] = frame;
        numframes++;

        if (numframes < numKeyframes) {
            continue;
        }

        if (keyframes[centerframe]->isGroup(mRelocalizer->LOOPCLOSUREFRAMEGROUP)) {
            continue;
        }

        if (!keyframes[centerframe - 1]->isGroup(INDIRECTKEYFRAME)) {
            continue;
        }

        if (!keyframes[centerframe + 1]->isGroup(INDIRECTKEYFRAME)) {
            continue;
        }

        int shared = keyframes[centerframe - 1]->sharedIndirect(keyframes[centerframe + 1]);
        if (shared > 100) { // todo : check the repartitions of the points
            keyframes[centerframe]->setGroup(INDIRECTKEYFRAME, false);
          //  keyframes[centerframe]->addDeform(keyframes[centerframe - 1], keyframes[centerframe + 1]);
        }



    }

}

List<PPoint> Hybrid::indirectCreateNewImmaturePointFromMatchings(const List<Matching> &matchings) {
    List<PPoint> newImmatures;
    Mutex newImmaturesMutex;

    int scaleConsistencyFailures = 0, frontFailures = 0, parallaxFailures = 0, finiteFailures = 0;

    #if CML_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (size_t i = 0; i < matchings.size(); i++) {

        const Matching &matching = matchings[i];

        if (matching.getMapPoint().isNotNull()) {
            // If we are here, this means that the matchings are not consistent
            // logger.warn("Matching for triangulation have map point");
            // matching.getFrameB()->setMapPoint(matching.getIndexB(matching.getFrameB()), matching.getMapPoint());
            // matching.getFrameA()->setMapPoint(matching.getIndexA(matching.getFrameA()), matching.getMapPoint());
        } else {

            scalar_t parallax = matching.getFrameA()->getCamera().parallax(matching.getFrameB()->getCamera(), matching.noAssertGetUndistortedA(0), matching.noAssertGetUndistortedB(0));
            bool parallaxCondition = parallax < 0.99998;

            if (!parallaxCondition) {
                parallaxFailures++;
                continue;
            }

            Vector3 pos = mTriangulator->triangulate(matching.getFrameA()->getCamera(), matching.getFrameB()->getCamera(), matching.getUndistortedA(matching.getFrameA(), 0), matching.getUndistortedB(matching.getFrameB(), 0));


            bool finiteCondition = pos.allFinite();
            bool frontCondition = matching.getFrameA()->getCamera().inFront(pos) && matching.getFrameB()->getCamera().inFront(pos);



            //Check scale consistency
            float dist1 = (pos - matching.getFrameA()->getCamera().eye()).norm();
            float dist2 = (pos - matching.getFrameB()->getCamera().eye()).norm();

            if(dist1==0 || dist2==0) {
                continue;
            }

            const float ratioDist = dist2/dist1;
            const float ratioOctave = matchings[i].noAssertGetFeaturePointA().processScaleFactorFromLevel() / matchings[i].noAssertGetFeaturePointB().processScaleFactorFromLevel();

            const float ratioFactor = 1.5f * matchings[i].noAssertGetFeaturePointA().scaleFactor();

            bool scaleConsistencyCondition = !(ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor);

            if (!scaleConsistencyCondition) {
                scaleConsistencyFailures++;
            }

            if(!frontCondition) {
                frontFailures++;
            }

            if (!finiteCondition) {
                finiteFailures++;
            }

            if (scaleConsistencyCondition && frontCondition && parallaxCondition && finiteCondition) {

                PPoint mapPoint = getMap().createMapPoint(matching.getFrameB(), matching.getIndexB(matching.getFrameB()),INDIRECT);

                auto frameAdata = get(matching.getFrameA());

                mapPoint->setDescriptor(frameAdata->descriptors[matching.getIndexA(matching.getFrameA()).index]);
                matching.getFrameB()->setMapPoint(matching.getIndexB(matching.getFrameB()), mapPoint);
                matching.getFrameA()->setMapPoint(matching.getIndexA(matching.getFrameA()), mapPoint);

                mapPoint->setWorldCoordinate(WorldPoint::fromAbsolute(pos));


                mapPoint->subscribeObserver(this);
                //updatePointDescriptor(mapPoint);

                mapPoint->setGroup(IMMATUREINDIRECTPOINT, true);
                mapPoint->setGroup(ACTIVEINDIRECTPOINT, true);

                LockGuard lg(newImmaturesMutex);
                newImmatures.emplace_back(mapPoint);

            }
        }

    }

    logger.important("Mapped " + std::to_string(newImmatures.size()) + " 3D points");
    logger.important("Scale consistency failures : " + std::to_string(scaleConsistencyFailures));
    logger.important("Front failures : " + std::to_string(frontFailures));
    logger.important("Parallax failures : " + std::to_string(parallaxFailures));
    logger.important("Finite failures : " + std::to_string(finiteFailures));


    return newImmatures;
}

List<PPoint> Hybrid::indirectCreateNewImmaturePoint(PFrame currentFrame) {

    if (!have(currentFrame)) {
        return {};
    }

    auto currentFrameData = get(currentFrame);

    if (currentFrameData->featureId < 0) {
        return {};
    }

    List<PPoint> newImmaturePoints;

    bool isFirst = true;

    List<PFrame> frames = getMap().processIndirectCovisiblity(currentFrame, 20, INDIRECTKEYFRAME);
    if (frames.size() == 0) {
        // frames = getMap().processDirectCovisiblity(currentFrame, 20);
        frames = getMap().getGroupFrameAsList(mPhotometricBA->ACTIVEKEYFRAME);
    }

    for (auto frame : frames) {

        if (frame == currentFrame) {
            continue;
        }

        if (!isFirst && mIndirectMappingQueue.getCurrentSize() > 1) {
            return newImmaturePoints;
        }

        if (!have(frame)) {
            continue;
        }

        auto frameData = get(frame);

        if (frameData->featureId < 0) {
            continue;
        }

        const float baseline = (currentFrame->getCamera().eye() - frame->getCamera().eye()).norm();
        const float medianDepth = frame->computeMedianDepth();
        const float ratioBaselineDepth = baseline / medianDepth;

        if(ratioBaselineDepth<0.01) {
            continue;
        }

        needVocabularyFor(frame);
        needVocabularyFor(currentFrame);
        List<Matching> matchings = mTriangulationTracker->trackForTriangulation({frame, frameData->featureId, frameData->descriptors},{currentFrame, currentFrameData->featureId, currentFrameData->descriptors});

        auto currentNewImmaturePoints = indirectCreateNewImmaturePointFromMatchings(matchings);

        newImmaturePoints.insert(newImmaturePoints.end(), currentNewImmaturePoints.begin(), currentNewImmaturePoints.end());

        isFirst = false;

    }

    return newImmaturePoints;

}

void Hybrid::indirectTrackImmature(PFrame currentFrame) {
    List<PPoint> toRemove;
    Mutex toRemoveMutex;

    List<PPoint> toMap;
    Mutex toMapMutex;

    auto immatureIndirectPoint = getMap().getGroupMapPointsAsList(IMMATUREINDIRECTPOINT);

    #if CML_USE_OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (size_t i = 0; i < immatureIndirectPoint.size(); i++) {

        auto &point = immatureIndirectPoint[i];

        int frameElapsed = currentFrame->getId() - point->getReferenceFrame()->getId();
        int keyFrameElapsed = currentFrame->getGroupId(INDIRECTKEYFRAME) - point->getReferenceFrame()->getGroupId(INDIRECTKEYFRAME);

        if (keyFrameElapsed < 3 || frameElapsed < 5) {
            continue;
        }

        scalar_t foundRatio = (scalar_t)point->getIndirectApparitionNumber() / (scalar_t)point->getDirectApparitionNumber();

        if (foundRatio < 0.25) {
            LockGuard lg(toRemoveMutex);
            toRemove.emplace_back(point);
            continue;
        }


        int indirectKeyApparition = 0;
        List<PFrame> frames;
        for (auto frame : point->getIndirectApparitions()) {
            if (!frame->isGroup(INDIRECTKEYFRAME)) {
                continue;
            }
            indirectKeyApparition++;
            frames.emplace_back(frame);
        }
        /*
        for (auto frameA : point->getIndirectApparitions()) {
            if (frameA->isGroup(INDIRECTKEYFRAME)) indirectKeyApparition++;
        }*/

        if (indirectKeyApparition >= 3) {

            bool canActivate;
            if (mIndirectUncertaintyThreshold.f() < 0) {
                if (mOptimiseOrbEachTime.b()) {
                    canActivate = mIndirectCeresBundleAdjustment->optimizeSinglePoint(point, frames, true);
                } else {
                    canActivate = true;
                }
            } else {
                canActivate = mIndirectCeresBundleAdjustment->optimizeSinglePoint(point, frames, true) && point->getUncertainty() < mIndirectUncertaintyThreshold.f();
            }

            if (canActivate) {
                LockGuard lg(toMapMutex);
                toMap.emplace_back(point);
                continue;
            }
        }

        if (keyFrameElapsed > 10) {
            LockGuard lg(toRemoveMutex);
            toRemove.emplace_back(point);
            continue;
        }


    }

    for (auto point : toMap) {
        point->setGroup(IMMATUREINDIRECTPOINT, false);
    }

    for (auto point : toRemove) {
        getMap().removeMapPoint(point);
    }

}


void Hybrid::indirectSearchInNeighbors(PFrame currentFrame)
{
    // Retrieve neighbor keyframes
    int nn=20;

    Set<PFrame, Hasher> covisibleFrames;
    for(auto pKFi : getMap().processIndirectCovisiblity(currentFrame, nn, INDIRECTKEYFRAME))
    {
        covisibleFrames.insert(pKFi);
        for(auto frame : getMap().processIndirectCovisiblity(pKFi, 5, INDIRECTKEYFRAME))
        {
            covisibleFrames.insert(frame);
        }
    }
    covisibleFrames.erase(currentFrame);

    List<PPoint> currentframeMapPoints;
    for (auto point : currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP)) {
        if (currentFrame->getIndex(point).hasValidValue()) {
            currentframeMapPoints.emplace_back(point);
        }
    }

    for(auto frame : covisibleFrames)
    {
        auto frameData = get(frame);
        auto points = mTriangulationTracker->fuse(
                {frame, frameData->featureId, frameData->descriptors},
                currentframeMapPoints
        );
        for (auto point : points) {
            point->setGroup(POINTSTOUPDATE, true);
        }
    }


    Set<PPoint, Hasher> covisibleMapPointsSet;

    for(auto frame : covisibleFrames) {
        for (auto point : frame->getGroupMapPoints(getMap().INDIRECTGROUP)) {
            if (frame->getIndex(point).hasValidValue()) {
                covisibleMapPointsSet.insert(point);
            }
        }
    }
    for (auto point : currentframeMapPoints) {
        covisibleMapPointsSet.erase(point);
    }


    List<PPoint> covisibleMapPoints;
    for (auto point : covisibleMapPointsSet) {
        covisibleMapPoints.emplace_back(point);
    }

    auto currentFrameData = get(currentFrame);
    auto points = mTriangulationTracker->fuse(
            {currentFrame, currentFrameData->featureId, currentFrameData->descriptors},
            covisibleMapPoints
    );
    for (auto point : points) {
        point->setGroup(POINTSTOUPDATE, true);
    }

    for (auto point : getMap().getGroupMapPoints(POINTSTOUPDATE)) {
        updatePointDescriptor(point);
        point->setGroup(POINTSTOUPDATE, false);
    }

}

