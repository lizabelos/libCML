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
        mCornerExtractor->setNumFeatures(mNumOrbCorner.i() * mNumOrbMultiplier.f());
    } else {
        mCornerExtractor->setNumFeatures(mNumOrbCorner.i());
    }
/*
    List<Corner> corners;
    mCornerExtractor->compute(currentFrame->getCaptureFrame(), corners, currentFrameData->descriptors);
    currentFrameData->featureId = currentFrame->addFeaturePoints(corners);
    assertDeterministic("Number of ORB points extracted", corners.size());
    assertDeterministic("Hash of ORB extracted descriptors", computeHashOfDescriptors(currentFrameData->descriptors));
*/

#if CML_USE_OPENMP
    #pragma omp parallel
#endif
    mCornerExtractor->compute(currentFrame->getCaptureFrame());

    currentFrameData->descriptors = mCornerExtractor->getDescriptors();

    assertDeterministic("Number of ORB points extracted", currentFrameData->descriptors.size());

    if (mCornerExtractor->getCorners().size() > 0) {
        currentFrameData->featureId = currentFrame->addFeaturePoints(mCornerExtractor->getCorners());
    } else {
        assert(false);
    }

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
        CML_LOG_DEBUG("This is strange...");
        return Optional<Binary256Descriptor>();
    }
    if (descriptors.size() <= 2) {
        return descriptors[0];
    }
    auto d = computeDistinctiveDescriptors(descriptors);
    //return computeMedianDescriptors(descriptors);
    // assertDeterministic("Hash of distinctive descriptor", d.hash());

    return d;
}

bool Hybrid::indirectTrackWithCMLGraph(PFrame currentFrame) {

    Camera motionToTry = getMap().getLastFrame(1)->getCamera() * getMap().getLastFrame(2)->getCamera().to(getMap().getLastFrame(1)->getCamera());

    OptPFrame lastFrame = mLastFrame;
    OptPFrame lastKeyFrame = getMap().getLastGroupFrame(CML::AbstractSlam::getMap().KEYFRAME);

    if (lastFrame.isNull()) {
        return false;
    }

    if (lastKeyFrame.isNull()) {
        return false;
    }

    if (!have(lastFrame)) {
        return false;
    }

    if (!have(lastKeyFrame)) {
        return false;
    }

    if (lastFrame == lastKeyFrame) {
        lastKeyFrame = getMap().getLastGroupFrame(CML::AbstractSlam::getMap().KEYFRAME, 2);
    }

    if (lastKeyFrame.isNull()) {
        return false;
    }

    if (!have(lastKeyFrame)) {
        return false;
    }

    PointSet activePoints = getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT);

    HashMap<PPoint, Eigen::SparseVector<scalar_t>> scores;

    int currentFeatureId = get(currentFrame)->featureId;
    List<Corner> currentCorners = currentFrame->getFeaturePoints(currentFeatureId);
    List<Descriptor> &currentDescriptors = get(currentFrame)->descriptors;
    int currentCornersNum = currentDescriptors.size();


    List<Descriptor> &lastFrameDescriptors = get(lastFrame)->descriptors;
    List<Descriptor> &lastKeyFrameDescriptors = get(lastKeyFrame)->descriptors;

    List<NearestNeighbor> nearestNeighbor;

    for (PPoint point : activePoints) {

        DistortedVector2d projection = currentFrame->distort(point->getWorldCoordinate().project(currentFrame->getCamera()), 0);

        // currentFrame->processNearestNeighbors(currentFeatureId, projection, 20, nearestNeighbor); // todo : 20 is a parameter
        currentFrame->processNearestNeighborsInRadius(currentFeatureId, projection, 14, nearestNeighbor); // todo : 20 is a parameter

        for (auto &nn : nearestNeighbor) {

            DistortedVector2d projection;
            int nPredictedLevel;
            scalar_t viewCos;
            if (!Features::computeViewcosAndScale(point, currentFrame, motionToTry, projection, viewCos, nPredictedLevel)) {
                continue;
            }

            // matching.set(nn.index, i, descriptorsB[i].distance(descriptorsA[nn.index]));
            scalar_t distScore = 0;
            if (nn.distance <= 1.0) {
                distScore = 99999;
            } else {
                distScore = 1000.0 / nn.distance;
            }
            //scalar_t descriptorScore = 1000.0 / currentDescriptors[nn.index].distance(point->getDescriptor<Binary256Descriptor>());
            scalar_t descriptorScore = 0;
            scalar_t levelScore = 1000.0 / (1 + std::abs(nPredictedLevel - currentCorners[nn.index].level()));

            FeatureIndex lastFrameIndex = lastFrame->getIndex(point);
            if (lastFrameIndex.hasValidValue()) {
                scalar_t dist = currentDescriptors[nn.index].distance(lastFrameDescriptors[lastFrameIndex.index]);
                if (dist < 1.0) {
                    descriptorScore += 99999;
                } else {
                    descriptorScore += 1000.0 / dist;
                }
            }

            FeatureIndex lastKeyFrameIndex = lastKeyFrame->getIndex(point);
            if (lastKeyFrameIndex.hasValidValue()) {
                scalar_t dist = currentDescriptors[nn.index].distance(lastKeyFrameDescriptors[lastKeyFrameIndex.index]);
                if (dist < 1.0) {
                    descriptorScore += 99999;
                } else {
                    descriptorScore += 1000.0 / dist;
                }
            }

            scalar_t score = distScore + descriptorScore + levelScore;

            if (score < 2000) {
                continue;
            }

            if (scores.find(point) == scores.end()) {
                //scores[point] = List<int>(currentCornersNum, 0);
                scores[point].resize(currentCornersNum); // todo : use a sparse matrix ?
            }

            // compute maxScore
            scalar_t maxScore = 0;
            for (Eigen::SparseVector<double>::InnerIterator it(scores[point]); it; ++it) {
                if (it.value() > maxScore) {
                    maxScore = it.value();
                }
            }

            if (score > maxScore * 0.8) {
                scores[point].coeffRef(nn.index) += score;
                scores[point].prune(score * 0.8);
            }

        }

    }

    List<Matching> matchings;
    List<bool> outliers;


    for (auto &score : scores) {
        PPoint point = score.first;
        auto &pointScores = score.second;

        int maxScore = 0;
        for (Eigen::SparseVector<double>::InnerIterator it(pointScores); it; ++it) {
            scalar_t score = it.value();
            if (score > maxScore) {
                maxScore = score;
            }
        }

        for (Eigen::SparseVector<double>::InnerIterator it(pointScores); it; ++it) {
            scalar_t score = it.value();
            if (score > maxScore * 0.8) {
                // scalar_t descriptorDistance, PFrame frameA, PFrame frameB, FeatureIndex indexA, FeatureIndex indexB
                PFrame frameB = *point->getIndirectApparitions().begin();
                Matching matching(1.0 / score, currentFrame, frameB, FeatureIndex(currentFeatureId, it.index()), frameB->getIndex(point));
                matching.getMapPoint()->getWorldCoordinate();
                matchings.emplace_back(matching);

                if (score == maxScore) {
                    outliers.emplace_back(false);
                } else {
                    outliers.emplace_back(true);
                }

            }

        }

    }



    if (matchings.size() < 20) {
        CML_LOG_IMPORTANT("Not accepting the tracking with motion model because of the number of matchings");
        return false;
    }

    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, motionToTry, matchings, outliers, mTrackcondUncertaintyWeight.f() > 0);
    if (!mLastIndirectTrackingResult.isOk) {
        CML_LOG_IMPORTANT("Not accepting the tracking with motion model because the optimization failed");
        return false;
    }



    mLastOrbTrackingInliersRatio = 0.5; // todo : hack

    assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

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



}

bool Hybrid::indirectTrackWithMotionModel(PFrame currentFrame, Optional<Camera> optionalMotionToTry) {
    mLastHaveSucceedCVMM = 1;
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
    CML_LOG_INFO("Found " + std::to_string(matchings.size()) + " matchings from last frame");

    if (matchings.size() < 20) {
        CML_LOG_IMPORTANT("Not accepting the tracking with motion model because of the number of matchings");
        return false;
    }

    List<bool> outliers;
    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, motionToTry, matchings, outliers, mTrackcondUncertaintyWeight.f() > 0);
    if (!mLastIndirectTrackingResult.isOk) {
        CML_LOG_IMPORTANT("Not accepting the tracking with motion model because the optimization failed");
        return false;
    }

    int numInliers = 0;

    for (size_t i = 0; i < matchings.size(); i++) {
        if (outliers[i]) {
            continue;
        }
        numInliers++;
    }

    float inliersRatio = (float)numInliers / (float)matchings.size();
    mLastOrbTrackingInliersRatio = inliersRatio;

    assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

    if (numInliers >= mOrbInlierNumThreshold.i() && inliersRatio > mOrbInlierRatioThreshold.f()) {
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
        CML_LOG_IMPORTANT("Not accepting the tracking with motion model because too many outliers");
        return false;
    }
}

bool Hybrid::indirectTrackReferenceKeyFrame(PFrame currentFrame) {
    mLastHaveSucceedCVMM = 0;
    OptPFrame referenceKeyFrame = mReferenceKeyFrame;

    if (referenceKeyFrame.isNull()) {
        CML_LOG_IMPORTANT("No reference keyframe");
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

    CML_LOG_INFO("Found " + std::to_string(matchings.size()) + " matchings from reference");
    assertDeterministic("Number of matchings for indirect tracking with motion model", matchings.size());


    if (matchings.size() < 15) {
        CML_LOG_IMPORTANT("Not enough matchings");
        return false;
    }

    List<bool> outliers;
    mLastIndirectTrackingResult = mPnP->optimize(currentFrame, mLastFrame->getCamera(), matchings, outliers, mTrackcondUncertaintyWeight.f() > 0);
    if (!mLastIndirectTrackingResult.isOk) {
        CML_LOG_IMPORTANT("Reference PnP failed");
        return false;
    }

    int numInliers = 0;

    for (size_t i = 0; i < matchings.size(); i++) {
        if (outliers[i]) {
            continue;
        }
        numInliers++;
    }
    float inliersRatio = (float)numInliers / (float)matchings.size();
    mLastOrbTrackingInliersRatio = inliersRatio;

    assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

    if (numInliers >= mOrbInlierNumThreshold.i() && inliersRatio > mOrbInlierRatioThreshold.f()) {
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
        CML_LOG_IMPORTANT("Too few inliers from reference : " + std::to_string(numInliers));
        return false;
    }

}

bool Hybrid::indirectTrackLocalMap(PFrame currentFrame) {

    indirectUpdateLocalKeyFrames(currentFrame);
    indirectUpdateLocalPoints(currentFrame);
    indirectSearchLocalPoints(currentFrame);

    if (mTrackcondUncertaintyWeight.f() > 0) {

        List<PPoint> outliers;
        mLastIndirectTrackingResult = mPnP->optimize(currentFrame, outliers, mTrackcondUncertaintyWeight.f() > 0);

        if (!mTrackedWithDirect || mLastPhotometricTrackingResidual.saturatedRatio() >= 0.15) {
            assertDeterministic("Indirect covariance norm", mLastIndirectTrackingResult.covariance.norm());
            if (mLastIndirectTrackingResult.isOk) {

                int numInliers = currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP).size() - outliers.size();
                float inliersRatio =
                        numInliers / (float) currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP).size();

                assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

                if (numInliers >= 10 && inliersRatio > mOrbInlierRatioThreshold.f()) {

                    currentFrame->setCamera(mLastIndirectTrackingResult.camera);
                    assertDeterministicMsg("Refining with ORB");
                }
            }
        }

    } else {


        if (!mTrackedWithDirect || mLastPhotometricTrackingResidual.saturatedRatio() >= 0.15) {
            assertDeterministic("Indirect covariance norm", mLastIndirectTrackingResult.covariance.norm());
            List<PPoint> outliers;
            mLastIndirectTrackingResult = mPnP->optimize(currentFrame, outliers, mTrackcondUncertaintyWeight.f() > 0);

            if (mLastIndirectTrackingResult.isOk) {

                int numInliers = currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP).size() - outliers.size();
                float inliersRatio =
                        numInliers / (float) currentFrame->getGroupMapPoints(getMap().INDIRECTGROUP).size();

                assertDeterministic("Number of inliers for indirect tracking with motion model", numInliers);

                if (numInliers >= 10 && inliersRatio > mOrbInlierRatioThreshold.f()) {

                    currentFrame->setCamera(mLastIndirectTrackingResult.camera);
                    assertDeterministicMsg("Refining with ORB");
                }
            }
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
        CML_LOG_ERROR("Indirect search local points : no features...");
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
        CML_LOG_ERROR("Update local key frames : no features ?");
        return;
    }

    FrameHashMap<int> keyframeCounter;
    keyframeCounter.reserve(100);

    List<PFrame> tmpIndirectApparitions;
    tmpIndirectApparitions.reserve(1000);

    size_t featurePointsSize = currentFrame->getFeaturePoints(currentFrameData->featureId).size();

    for(size_t i = 0; i < featurePointsSize; i++)
    {
        OptPPoint pMP = currentFrame->getMapPoint(FeatureIndex(currentFrameData->featureId, i));
        if (pMP.isNull()) {
            continue;
        }
        pMP->getIndirectApparitions(tmpIndirectApparitions);
        for(auto frame : tmpIndirectApparitions) {
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
    FrameSet localKeyFrames = mLocalKeyFrames; // todo : recursive ??
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
    PointSet toUnactive = getMap().getGroupMapPoints(ACTIVEINDIRECTPOINT), toActive;

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
        CML_LOG_ERROR("Indirect new need key frame : no features...");
        return false;
    }

    if (!mTrackingOk && mOrbKeyframeSkipOnFailure.b()) {
        return false;
    }

    if (mReferenceKeyFrame.isNull()) {
        return true;
    }

    if (mReferenceKeyFrame == currentFrame) {
        return true;
    }

    int numTrackedRef = indirectNumTrackedRef();

    if (numTrackedRef > mOrbKeyframeReflimit.i()) {
        numTrackedRef = mOrbKeyframeReflimit.i();
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //const bool c1a = currentFrame->getId() >= getMap().getLastGroupFrame(INDIRECTKEYFRAME)->getId() + 20;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //const bool c1b =  currentFrame->getId() >= getMap().getLastGroupFrame(INDIRECTKEYFRAME)->getId() && mIndirectMappingQueue.getCurrentSize() == 0;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    //scalar_t threshold = exp(log(numTrackedRef) * 0.975);

    scalar_t threshold = numTrackedRef * mOrbKeyframeRatio.f();
    CML_LOG_INFO("Num tracked ref : " + std::to_string(numTrackedRef));
    CML_LOG_INFO("Indirect keyframe threshold : " + std::to_string(threshold));
    CML_LOG_INFO("Last Num Tracked : " + std::to_string(mLastNumTrackedPoints));

    /*if (mLastNumTrackedPoints > 200) {
        return false;
    }*/

    if (mOrbKeyframeMinimumPoints.i() >= 0 && mLastNumTrackedPoints < mOrbKeyframeMinimumPoints.i()) {
        return false;
    }

    return mLastNumTrackedPoints < threshold;

}