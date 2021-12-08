#include "Hybrid.h"

void Hybrid::extractOrb(PFrame currentFrame) {
    if (!mEnableIndirect.b()) {
        return;
    }
    auto currentFrameData = get(currentFrame);
    if (currentFrameData->featureId >= 0) {
        return;
    }

    if (mLastNumTrackedPoints < 15 || mState == NOT_INITIALIZED) {
        mCornerExtractor->setNumFeatures(mNumOrbCorner.i() * 2);
    } else {
        mCornerExtractor->setNumFeatures(mNumOrbCorner.i());
    }

    List<Corner> corners;
    mCornerExtractor->compute(currentFrame->getCaptureFrame(), corners, currentFrameData->descriptors);
    currentFrameData->featureId = currentFrame->addFeaturePoints(corners);
    assertDeterministic("Number of ORB points extracted", corners.size());
    assertDeterministic("Hash of ORB extracted descriptors", computeHashOfDescriptors(currentFrameData->descriptors));

/*
    mCornerExtractor->compute(currentFrame->getCaptureFrame());
    currentFrameData->descriptors = mCornerExtractor->getDescriptors();

    assertDeterministic("Number of ORB points extracted", currentFrameData->descriptors.size());

    if (mCornerExtractor->getCorners().size() > 0) {
        currentFrameData->featureId = currentFrame->addFeaturePoints(mCornerExtractor->getCorners());
    } else {
        assert(false);
    }
*/
}

void Hybrid::needVocabularyFor(PFrame currentFrame) {
    auto currentFrameData = get(currentFrame);

    LockGuard lg(currentFrameData->bowMutex);

    if (currentFrame->getBoW(currentFrameData->featureId).isNotNull()) {
        return;
    }

    Ptr<Features::BoW, Nullable> bow = new Features::BoW;
    mCornerExtractor->getVocabulary().transform(currentFrameData->descriptors, bow->bowVec, bow->featVec, 4);
    currentFrame->setBow(currentFrameData->featureId, bow);
}

Optional<Binary256Descriptor> Hybrid::findDescriptor(PPoint point) {
    List<Binary256Descriptor> descriptors;
    for (auto frame : point->getIndirectApparitions()) {
        if (have(frame)) {
            auto frameData = get(frame);
            FeatureIndex index = frame->getIndex(point);

            if (index.hasValidValue()) {
                descriptors.emplace_back(frameData->descriptors[index.index]);
            }
        }
    }
    if (descriptors.size() == 0) {
        logger.debug("This is strange...");
        return Optional<Binary256Descriptor>();
    }
    if (descriptors.size() <= 2) {
        return descriptors[0];
    }
    auto d = computeDistinctiveDescriptors(descriptors);
    //return computeMedianDescriptors(descriptors);
    assertDeterministic("Hash of distinctive descriptor", d.hash());

    return d;
}


bool Hybrid::indirectTrackWithMotionModel(PFrame currentFrame, Optional<Camera> optionalMotionToTry) {
    auto currentFrameData = get(currentFrame);
    auto lastFrameData = get(mLastFrame);

    Camera motionToTry;

    if (optionalMotionToTry.has_value()) {
        motionToTry = optionalMotionToTry.value();
    } else {
        motionToTry = getMap().getLastFrame(1)->getCamera() * getMap().getLastFrame(2)->getCamera().to(getMap().getLastFrame(1)->getCamera());
    }

    int th = 7;
    List<Matching> matchings = mMotionModelTracker->trackByProjection(
            {currentFrame, currentFrameData->featureId, currentFrameData->descriptors, motionToTry},
            {mLastFrame, lastFrameData->featureId},
            th
    );

    if (matchings.size() < 20) {
        matchings = mMotionModelTracker->trackByProjection(
                {currentFrame, currentFrameData->featureId, currentFrameData->descriptors, motionToTry},
                {mLastFrame, lastFrameData->featureId},
                2 * th
        );
    }

    assertDeterministic("Number of matching for indirect tracking with motion model", matchings.size());
    logger.important("Found " + std::to_string(matchings.size()) + " matchings from last frame");

    if (matchings.size() < 20) {
        logger.important("Not accepting the tracking with motion model because of the number of matchings");
        return false;
    }

    List<bool> outliers;
    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, motionToTry, matchings, outliers);
    if (!mLastIndirectTrackingResult.isOk) {
        logger.important("Not accepting the tracking with motion model because the optimization failed");
        return false;
    }

    int numInliers = 0;

    for (size_t i = 0; i < matchings.size(); i++) {
        if (outliers[i]) {
            continue;
        }
        numInliers++;
    }

    assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

    if (numInliers >= 10) {
        currentFrame->setCamera(mLastIndirectTrackingResult.camera);
        for (size_t i = 0; i < matchings.size(); i++) {
            if (outliers[i]) {
                continue;
            }
            if (currentFrame->setMapPoint(matchings[i].getIndexA(currentFrame), matchings[i].getMapPoint())) {
                currentFrame->addDirectApparitions(matchings[i].getMapPoint());
            }
        }
        return true;
    } else {
        logger.important("Not accepting the tracking with motion model because too many outliers");
        return false;
    }
}

bool Hybrid::indirectTrackReferenceKeyFrame(PFrame currentFrame) {

    OptPFrame referenceKeyFrame = mReferenceKeyFrame;

    if (referenceKeyFrame.isNull()) {
        logger.important("No reference keyframe");
        return false;
    }

    auto currentFrameData = get(currentFrame);
    auto referenceKeyFrameData = get(referenceKeyFrame);

    needVocabularyFor(currentFrame);
    needVocabularyFor(referenceKeyFrame);

    List<Matching> matchings = mReferenceTracker->trackByBoW(
            {currentFrame, currentFrameData->featureId, currentFrameData->descriptors},
            {referenceKeyFrame, referenceKeyFrameData->featureId, referenceKeyFrameData->descriptors}
    );

    logger.important("Found " + std::to_string(matchings.size()) + " matchings from reference");
    assertDeterministic("Number of matchings for indirect tracking with motion model", matchings.size());


    if (matchings.size() < 15) {
        logger.important("Not enough matchings");
        return false;
    }

    List<bool> outliers;
    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, mLastFrame->getCamera(), matchings, outliers);
    if (!mLastIndirectTrackingResult.isOk) {
        logger.important("Reference PnP failed");
        return false;
    }

    int numInliers = 0;

    for (size_t i = 0; i < matchings.size(); i++) {
        if (outliers[i]) {
            continue;
        }
        numInliers++;
    }

    assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

    if (numInliers >= 10) {
        currentFrame->setCamera(mLastIndirectTrackingResult.camera);
        for (size_t i = 0; i < matchings.size(); i++) {
            if (outliers[i]) {
                continue;
            }
            if (currentFrame->setMapPoint(matchings[i].getIndexA(currentFrame), matchings[i].getMapPoint())) {
                currentFrame->addDirectApparitions(matchings[i].getMapPoint());
            }
        }
        return true;
    } else {
        logger.important("Too few inliers from reference : " + std::to_string(numInliers));
        return false;
    }

}

bool Hybrid::indirectTrackLocalMap(PFrame currentFrame) {

    indirectUpdateLocalKeyFrames(currentFrame);
    indirectUpdateLocalPoints(currentFrame);
    indirectSearchLocalPoints(currentFrame);

    List<PPoint> outliers;
    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, outliers);

    if (!mTrackedWithDirect || mLastPhotometricTrackingResidual.saturatedRatio() >= 0.15) {
        assertDeterministic("Indirect covariance norm", mLastIndirectTrackingResult.covariance.norm());
        if (mLastIndirectTrackingResult.isOk) {
            currentFrame->setCamera(mLastIndirectTrackingResult.camera);
            assertDeterministic("Refining with ORB");
        }
    }

    int numTrackedPoints = 0;
    for (auto point : currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP)) {
        if (currentFrame->getIndex(point).hasValidValue()) {
            numTrackedPoints++;
        }
    }

    if (mFirstTrackingMatchingNumber < 0) {
        mFirstTrackingMatchingNumber = numTrackedPoints;
    }
    mLastNumTrackedPoints = numTrackedPoints;

    return numTrackedPoints > 30;
}


void Hybrid::indirectSearchLocalPoints(PFrame currentFrame) {

    auto currentFrameData = get(currentFrame);
    if (currentFrameData->featureId == -1) {
        logger.error("Indirect search local points : no features...");
        return;
    }

    for (auto point : getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT)) {

        assertThrow(point->isGroup(getMap().INDIRECTGROUP), "This is not an indirect point : " + std::to_string(point->getId()));

        if (currentFrame->getFeaturePoint(point).has_value()) {
            continue;
        }

        if (Features::isInFrustum(point, currentFrame)) {
            currentFrame->addDirectApparitions(point);
        }

    }

    int th = 1;
    // If the camera has been relocalised recently, perform a coarser search
    if(mLastRelocFrame.isNotNull() && currentFrame->getId() < mLastRelocFrame->getId() + 2) {
        th = 5;
    }

    List<PPoint> mapPoints = getMap().getGroupMapPointsAsList(ACTIVEINDIRECTPOINT);

    List<Pair<int, int>> matchings = mLocalPointsTracker->trackByProjection(
            {currentFrame, currentFrameData->featureId, currentFrameData->descriptors},
            mapPoints,th);

    for (auto [frameIndex, pointIndex] : matchings) {
        currentFrame->setMapPoint(FeatureIndex(currentFrameData->featureId, frameIndex), mapPoints[pointIndex]);
        currentFrame->addDirectApparitions(mapPoints[pointIndex]);
        updatePointDescriptor(mapPoints[pointIndex]);
    }

}

void Hybrid::indirectUpdateLocalKeyFrames(PFrame currentFrame) {
    // Each map point vote for the keyframes in which it has been observed
    auto currentFrameData = get(currentFrame);
    if (currentFrameData->featureId == -1) {
        logger.error("Update local key frames : no features ?");
        return;
    }
    HashMap<PFrame, int, Hasher> keyframeCounter;
    for(size_t i = 0; i < currentFrame->getFeaturePoints(currentFrameData->featureId).size(); i++)
    {
        OptPPoint pMP = currentFrame->getMapPoint(FeatureIndex(currentFrameData->featureId, i));
        if (pMP.isNull()) {
            continue;
        }
        for(auto frame : pMP->getIndirectApparitions()) {
            if (!frame->isGroup(INDIRECTKEYFRAME)) {
                continue;
            }
            keyframeCounter[frame]++;
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    OptPFrame pKFmax;

    mLocalKeyFrames.clear();

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(auto [pKF, count] : keyframeCounter)
    {
        if(count>max)
        {
            max = count;
            pKFmax=pKF;
        }

        mLocalKeyFrames.insert(pKF);
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    Set<PFrame, Hasher> localKeyFrames = mLocalKeyFrames; // todo : recursive ??
    for (auto pKF : localKeyFrames)
    {
        // Limit the number of keyframes
        if(mLocalKeyFrames.size() > 80) {
            break;
        }

        auto vNeighs = getMap().processIndirectCovisiblity(pKF, 10, INDIRECTKEYFRAME);

        for(auto pNeighKF : vNeighs)
        {
            if(mLocalKeyFrames.find(pNeighKF) == mLocalKeyFrames.end())
            {
                mLocalKeyFrames.insert(pNeighKF);
                break;
            }
        }

    }

    if(pKFmax.isNotNull())
    {
        mReferenceKeyFrame = pKFmax;
    }
}

void Hybrid::indirectUpdateLocalPoints(PFrame currentFrame) {
    Set<PPoint, Hasher> toUnactive = getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT), toActive;

    for (auto pKF : mLocalKeyFrames)
    {
        for (auto point : pKF->getGroupMapPoints(getMap().INDIRECTGROUP)) {
            toActive.insert(point);
        }
    }

    for (auto point : toActive) {
        toUnactive.erase(point);
    }

    for (auto point : toUnactive) {
        if (!point->isGroup(IMMATUREINDIRECTPOINT)) {
            point->setGroup(ACTIVEINDIRECTPOINT, false);
        }
    }


    for (auto point : toActive) {
        point->setGroup(ACTIVEINDIRECTPOINT, true);
    }
}

int Hybrid::indirectNumTrackedRef() {
    int numTrackedRef = 0;
    for (auto point : mReferenceKeyFrame->getGroupMapPoints(getMap().INDIRECTGROUP)) {
        if (!mReferenceKeyFrame->getIndex(point).hasValidValue()) {
            continue;
        }
        if (point->getIndirectApparitionNumber() >= 3) {
            numTrackedRef++;
        }
    }
    return numTrackedRef;
}


bool Hybrid::indirectNeedNewKeyFrame(PFrame currentFrame) {

    auto currentFrameData = get(currentFrame);
    if (currentFrameData->featureId == -1) {
        logger.error("Indirect new need key frame : no features...");
        return false;
    }

    if (!mTrackingOk) {
        return false;
    }

    if (mReferenceKeyFrame.isNull()) {
        return true;
    }

    if (mReferenceKeyFrame == currentFrame) {
        return true; // todo : check this
    }

    int numTrackedRef = indirectNumTrackedRef();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //const bool c1a = currentFrame->getId() >= getMap().getLastGroupFrame(INDIRECTKEYFRAME)->getId() + 20;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //const bool c1b =  currentFrame->getId() >= getMap().getLastGroupFrame(INDIRECTKEYFRAME)->getId() && mIndirectMappingQueue.getCurrentSize() == 0;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    scalar_t threshold = exp(log(numTrackedRef) * 0.975);
    logger.important("Num tracked ref : " + std::to_string(numTrackedRef));
    logger.important("Indirect keyframe threshold : " + std::to_string(threshold));
    logger.important("Last Num Tracked : " + std::to_string(mLastNumTrackedPoints));

    /*if (mLastNumTrackedPoints > 200) {
        return false;
    }*/

    const bool c2a = mLastNumTrackedPoints < threshold && mLastNumTrackedPoints > 15;

    //const bool c2b = mLastNumTrackedPoints < 15 && mTrackedWithDirect;

    return c2a;

}