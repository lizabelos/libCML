//
// Created by tbelos on 19/04/19.
//
#include "cml/map/Map.h"
#include "cml/evaluation/Alignment.h"

#include <iomanip>

CML::Map::Map() {
    //mObservers.set_empty_key((Observer*)1);
    //mObservers.set_deleted_key((Observer*)2);

    mFrameCounter = 0;
    mFrameKeyCounter = 0;
    mMapObjectCounter = 0;

    mBufferCoordinates.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE * 3]);
    mBufferColors.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE * 3]);
    mBufferGroups.emplace_back(new unsigned int[CML_MAP_MAPPOINT_BUFFER_SIZE]);
    mBufferUncertainty.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE]);
    memset(mBufferGroups[0], 0, CML_MAP_MAPPOINT_BUFFER_SIZE * sizeof(unsigned int));

    mBufferFrameCenter.emplace_back(new scalar_t[CML_MAP_FRAME_BUFFER_SIZE * 3]);
    mBufferFrameCenterSize.emplace_back(0);

    if (MAPPED != 0) {
        abort();
    }
}

CML::Map::~Map() {

    Set<PPoint, Hasher> points = mMapPoints;
    for (auto point : points) {
        removeMapPoint(point);
    }

    for (auto frame : mFrames) {
        mGarbageCollector.erase(frame.p());
    }

    for (auto buffer : mBufferCoordinates) {
        delete [] buffer;
    }

    for (auto buffer : mBufferColors) {
        delete [] buffer;
    }

    for (auto buffer : mBufferGroups) {
        delete [] buffer;
    }

    for (auto buffer : mBufferUncertainty) {
        delete [] buffer;
    }

}

CML::PPoint CML::Map::createMapPoint(PFrame reference, FeatureIndex referenceIndex, MapPointType type)
{
    assertThrow(reference != nullptr, "called createMapPoint with a null reference pointer");
    // assertThrow(reference->isInside(corner.point(0), 0, 0), "corner is not inside the reference");
    // PPoint object = mMapPointAllocator.allocate(1);
    // assertThrow(object != nullptr, "Failec to allocate mapObject");
    // mMapPointAllocator.construct(object.p(), reference, corner, type);
    size_t id;

    {
        LockGuard lg(mMapObjectAvailableIdsMutex);
        if (mMapObjectAvailableIds.empty()) {
            id = mMapObjectCounter++;
        } else {
            id = mMapObjectAvailableIds.front();
            mMapObjectAvailableIds.pop_front();
        }
    }

    size_t bufferId;
    size_t rowId;
    {
        LockGuard lg(mBuffersMutex);
        bufferId = id / CML_MAP_MAPPOINT_BUFFER_NUM;
        rowId = id % CML_MAP_MAPPOINT_BUFFER_NUM;
        if (bufferId >= mBufferCoordinates.size()) {
            mBufferCoordinates.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE * 3]);
            mBufferColors.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE * 3]);
            mBufferGroups.emplace_back(new unsigned int[CML_MAP_MAPPOINT_BUFFER_SIZE]);
            mBufferUncertainty.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE]);
            memset(mBufferGroups[bufferId], 0, CML_MAP_MAPPOINT_BUFFER_SIZE * sizeof(unsigned int));
        }
    }

    PPoint object = new MapPoint(id, reference, referenceIndex, type,
                                                     &mBufferCoordinates[bufferId][rowId * 3 * CML_MAPPOINT_SPARSITY],
                                                     &mBufferColors[bufferId][rowId * 3 * CML_MAPPOINT_SPARSITY],
                                                     &mBufferUncertainty[bufferId][rowId * CML_MAPPOINT_SPARSITY],
                                                     &mBufferGroups[bufferId][rowId * CML_MAPPOINT_SPARSITY]);
    object->subscribeObserver(this);

    {
        LockGuard lg(mMapPointsMutex);
        mMapPoints.insert(object);
    }

    object->setGroup(MAPPED, true);

    if (type == INDIRECT) {
        object->setGroup(INDIRECTGROUP, true);
    }

    if (type == DIRECT) {
        object->setGroup(DIRECTGROUP, true);
    }

    return object;
}

void CML::Map::removeMapPoint(PPoint mapPoint, bool singleHolder) {

    {
        LockGuard lg(mMapPointsMutex);
        auto search = mMapPoints.find(mapPoint);
        if (search == mMapPoints.end()) {
            return;
        }
        mMapPoints.erase(search);
    }

    assertThrow(mapPoint->isGroup(0), "Some strange bug");

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        mapPoint->setGroup(i, false);
    }

    assertThrow(!mapPoint->isGroup(0), "Some strange bug");

    Set<PFrame, Hasher> frames;
    for (auto frame : mapPoint->getDirectApparitions()) {
        frames.insert(frame);
    }
    for (auto frame : mapPoint->getIndirectApparitions()) {
        frames.insert(frame);
    }


  //  mapPoint->clearApparitions();
    for (auto frame : frames) {
        frame->onMapPointErased(mapPoint);
    }

    mapPoint->getPrivate().freeAll(mMapPointsPrivateDataContext, mGarbageCollector);

    if (singleHolder) {
        delete mapPoint.p();
    } else {
        mGarbageCollector.erase(mapPoint.p());
    }

    return;

}


CML::PFrame CML::Map::createFrame(Ptr<CaptureImage, NonNullable> captureFrame) {
    int id = mFrameCounter++;

    size_t bufferId;
    size_t rowId;
    {
        LockGuard lg(mBuffersMutex);
        bufferId = id / CML_MAP_FRAME_BUFFER_SIZE;
        rowId = id % CML_MAP_FRAME_BUFFER_SIZE;
        if (bufferId >= mBufferCoordinates.size()) {
            mBufferFrameCenter.emplace_back(new scalar_t[CML_MAP_MAPPOINT_BUFFER_SIZE * 3]);
            mBufferFrameCenterSize.emplace_back(0);
        }
    }

    PFrame frame = new Frame(id, captureFrame, &mBufferFrameCenter[bufferId][rowId * 3]);
    frame->setGroup(VALIDFRAME, true);

    mBufferFrameCenterSize[bufferId]++;

    return frame;
}

void CML::Map::addFrame(PFrame frame) {
    assertThrow(frame != nullptr, "called addFrame with a null pointer");
    {
        LockGuard lg(mFramesMutex);
        if (mFrames.count(frame) == 0) {
            mFrames.insert(frame);
            frame->subscribeObserver(this);
        }
    }
    {
        LockGuard lg(mObserversMutex);
        for (Observer *observer : mObservers) {
            observer->onAddFrame(*this, frame);
        }
    }
}

CML::Set<CML::PPoint, CML::Hasher> CML::Map::getMapPoints() {
    LockGuard lg(mMapPointsMutex);
    return mMapPoints;
}

int CML::Map::getMapPointsNumber() {
    return mMapPoints.size();
}

CML::Set<CML::PPoint, CML::Hasher> CML::Map::getGroupMapPoints(int groupId) {
    LockGuard lg(mGroupsMapPointMutexes[groupId]);
    return mGroupsMapPoint[groupId];
}

CML::OrderedSet<CML::PFrame, CML::Comparator> CML::Map::getFrames() {
    LockGuard lg(mFramesMutex);
    return mFrames;
}

CML::PFrame CML::Map::getLastFrame() {
    LockGuard lg(mFramesMutex);
    return *mFrames.begin();
}

CML::PFrame CML::Map::getLastFrame(int n) {
    LockGuard lg(mFramesMutex);
    auto it = mFrames.begin();
    std::advance(it, n);
    return *(it);
}

unsigned int CML::Map::getFramesNumber() {
    LockGuard lg(mFramesMutex);
    return mFrames.size();
}

unsigned int CML::Map::getGroupFramesNumber(int groupId) {
    LockGuard lg(mGroupsFrameMutexes[groupId]);
    return mGroupsFrames[groupId].size();
}

CML::OptPFrame CML::Map::getLastGroupFrame(int groupId) {
    LockGuard lg(mGroupsFrameMutexes[groupId]);
    if (mGroupsFrames[groupId].empty()) {
        return {};
    }
    return *(mGroupsFrames[groupId].begin());
}

CML::OptPFrame CML::Map::getLastGroupFrame(int groupId, int n) {
    LockGuard lg(mGroupsFrameMutexes[groupId]);
    if (mGroupsFrames[groupId].empty()) {
        return {};
    }
    auto it = mGroupsFrames[groupId].begin();
    std::advance(it, n);
    return *(it);
}

CML::OrderedSet<CML::PFrame, CML::Comparator> CML::Map::getGroupFrames(int groupId) {
    if (groupId == -1) {
        return getFrames();
    }
    LockGuard lg(mGroupsFrameMutexes[groupId]);
    return mGroupsFrames[groupId];
}

void CML::Map::reset() {

    FrameSet frameSet;
    PointSet pointSet;

    {
        LockGuard lg(mFramesMutex);
        for (PFrame frame : mFrames) {
            frame->removeObserver(this);
            frameSet.insert(frame);
        }
        mFrames = OrderedSet<PFrame , Comparator>();
    }

    for (PPoint point : mMapPoints) {
        pointSet.insert(point);
    }
    mMapPoints.clear();

    for (int i = 0; i < FRAME_GROUP_MAXSIZE; i++) {
        mGroupsFrames[i] = OrderedSet<PFrame, Comparator>();
    }

    /*
    for (PFrame frame : frameSet) {
        mFrameAllocator.destroy(frame.p());
        mFrameAllocator.deallocate(frame.p(), 1);
    }

    for (PPoint mapObject : pointSet) {
        mMapPointAllocator.destroy(mapObject.p());
        mMapPointAllocator.deallocate(mapObject.p(), 1);
    }*/
    // TODO : Put this into a timed garbage collector
    
}

void CML::Map::onFrameGroupChange(PFrame frame, int groupId, bool state) {
    assertThrow(groupId >= 0 && groupId < FRAME_GROUP_MAXSIZE, "Max group out of range");

    if (groupId == KEYFRAME && state == true) {
        frame->getCaptureFrame().makeKey();
    }

  //  if (groupId == KEYFRAME && state == false) {
  //      frame->getCaptureFrame().makeUnactive();
  //  }

    {
        LockGuard lg(mGroupsFrameMutexes[groupId]);
        if (state) {
            mGroupsFrames[groupId].insert(frame);
        } else {
            mGroupsFrames[groupId].erase(frame);
        }
    }
    {
        LockGuard lg(mObserversMutex);
        for (Observer *observer : mObservers) {
            observer->onFrameChangeGroup(*this, frame, groupId, state);
        }
    }

    if (groupId == KEYFRAME && state == true) {
        frame->setKeyId(mFrameKeyCounter++);
        // refreshErrorFromGroundtruth();
    }

}

void CML::Map::onFrameDelete(CML::PFrame frame) {
    {
        LockGuard lg(mFramesMutex);
        mFrames.erase(frame);
    }
    
    for (int i = 0; i < FRAME_GROUP_MAXSIZE; i++) {
        LockGuard lg(mGroupsFrameMutexes[i]);
        mGroupsFrames[i].erase(frame);
    }
}

void CML::Map::onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) {

    assertThrow(groupId >= 0 && groupId < MAPOBJECT_GROUP_MAXSIZE, "Max group out of range");

    {
        LockGuard lg(mGroupsMapPointMutexes[groupId]);
        if (state) {
            mGroupsMapPoint[groupId].insert(mapPoint);
            assertThrow(mGroupsMapPoint[groupId].contains(mapPoint), "Some strange bug");
        } else {
            mGroupsMapPoint[groupId].erase(mapPoint);
            assertThrow(!mGroupsMapPoint[groupId].contains(mapPoint), "Some strange bug");
        }
    }
    {
        LockGuard lg(mObserversMutex);
        for (Observer *observer : mObservers) {
            observer->onMapPointChangeGroup(*this, mapPoint, groupId, state);
        }
    }

}

void CML::Map::onMapPointDestroyed(PPoint mapPoint) {

    {
        LockGuard lg(mMapObjectAvailableIdsMutex);
        mMapObjectAvailableIds.push_back(mapPoint->getId());
    }

}


CML::List<CML::PFrame> CML::Map::processIndirectCovisiblity(PFrame frame, int max, int groupId, int th) {

    //LockGuard lg(mTmpIndirectApparitionsMutex);

 //   mTmpIndirectApparitions.reserve(1000);

    auto candidates = frame->getIndirectCovisibilities();
   /* for (auto [index, point] : frame->getMapPoints()) {
        point->getIndirectApparitions(mTmpIndirectApparitions);
        for (auto covisibleFrame : mTmpIndirectApparitions) {
            if (groupId != -1 && !covisibleFrame->isGroup(groupId)) {
                continue;
            }
            if (candidates.count(covisibleFrame) == 0) {
                candidates.insert(Pair<PFrame, int>(covisibleFrame, 0));
            }
            candidates[covisibleFrame] += 1;
        }
    }*/
    if (candidates.size() == 0) {
        return List<PFrame>();
    }

    List<Pair<PFrame, int>> candidatesList;
    for (auto pair : candidates) {
        if (pair.first == frame) {
            continue;
        }
        if (pair.first->isGroup(groupId)) {
            candidatesList.emplace_back(pair);
        }
    }

    if (candidates.size() == 0) {
        return List<PFrame>();
    }

    std::sort(candidatesList.begin(), candidatesList.end(), [](const auto &a, const auto &b) {
        return a.second > b.second;
    });

    assertThrow(candidatesList[0].second >= candidatesList[candidatesList.size() - 1].second, "Ehh, std::sort problem");


    List<PFrame> result;


    for (auto [candidate, count] : candidatesList) {
        result.emplace_back(candidate);
        if (count < th) return result;
        if (max != -1 && (int)result.size() == max) return result;
    }
    return result;


}

CML::List<CML::PFrame> CML::Map::processDirectCovisiblity(PFrame frame, int max, int groupId) {

    //LockGuard lg(mTmpDirectApparitionsMutex);

  //  mTmpDirectApparitions.reserve(1000);

    // todo : we need to store the covisibility, store the graph

    auto candidates = frame->getDirectCovisibilities();
    /*for (auto point : frame->getMapPointsApparitions()) {
        point->getDirectApparitions(mTmpDirectApparitions);
        for (auto covisibleFrame : mTmpDirectApparitions) {
            if (groupId != -1 && !covisibleFrame->isGroup(groupId)) {
                continue;
            }
            if (candidates.count(covisibleFrame) == 0) {
                candidates.insert(Pair<PFrame, int>(covisibleFrame, 0));
            }
            candidates[covisibleFrame] += 1;
        }
    } */

    if (candidates.size() == 0) {
        return List<PFrame>();
    }

    List<Pair<PFrame, int>> candidatesList;
    for (auto pair : candidates) {
        if (pair.first == frame) {
            continue;
        }
        if (pair.first->isGroup(groupId)) {
            candidatesList.emplace_back(pair);
        }
    }

    if (candidates.size() == 0) {
        return List<PFrame>();
    }

    std::sort(candidatesList.begin(), candidatesList.end(), [](const auto &a, const auto &b) {
        return a.second > b.second;
    });

    assertThrow(candidatesList[0].second >= candidatesList[candidatesList.size() - 1].second, "Ehh, std::sort problem");


    List<PFrame> result;


    for (auto [candidate, count] : candidatesList) {
        result.emplace_back(candidate);
        if (max != -1 && (int)result.size() == max) return result;
    }
    return result;


}


void CML::Map::refreshErrorFromGroundtruth() {

    List<Optional<Camera>> cameras;
    List<Camera> groundtruth;

    for (auto frame : getFrames()) {
        if (frame->getCaptureFrame().getGroundtruth().has_value()) {
            cameras.emplace_back(frame->getCamera());
            groundtruth.emplace_back(frame->getCaptureFrame().getGroundtruth().value());
        }
    }

    Evaluation::align(groundtruth, cameras, groundtruth);

    LockGuard lg(mErrorMutex);
    mAlignedGronudtruth = groundtruth;

}

void CML::Map::exportResults(std::string path, MapResultFormat format, bool align) {

    logger.important("Writing the results to " + path);

    List<Camera> cameras;
    List<Optional<Camera>> groundtruth;
    List<bool> iskeyframe;

    for (auto frame : getFrames()) {
        cameras.emplace_back(frame->getCamera());
        groundtruth.emplace_back(frame->getCaptureFrame().getGroundtruth());
        iskeyframe.emplace_back(frame->isGroup(KEYFRAME));
    }

    std::reverse(cameras.begin(), cameras.end());
    std::reverse(groundtruth.begin(), groundtruth.end());
    std::reverse(iskeyframe.begin(), iskeyframe.end());

    if (align) {
        Evaluation::align(cameras, groundtruth, cameras);
    }

    logger.important("Writing " + std::to_string(cameras.size()) + " poses");



    if (format == MAP_RESULT_FORMAT_TUM) {

        logger.important("Writing with TUM format");


        std::ofstream file;
        file.open(path);
        file << std::setprecision(15);

        for (auto frame : getFrames())
        {
            file << frame->getCaptureFrame().getEvaluationTimestamp() <<
                   " " << frame->getCamera().getTranslation()(0)<<
                   " " << frame->getCamera().getTranslation()(1)<<
                   " " << frame->getCamera().getTranslation()(2)<<
                   " " << frame->getCamera().getQuaternion().x()<<
                   " " << frame->getCamera().getQuaternion().y()<<
                   " " << frame->getCamera().getQuaternion().z()<<
                   " " << frame->getCamera().getQuaternion().w() << "\n";
        }

        file.close();

    }

    if (format == MAP_RESULT_FORMAT_KITTI) {

        logger.important("Writing with KITTI format");

        std::ofstream file;
        file.open(path);
        file << std::setprecision(15);

        scalar_t ateAllframesRMSE = 0, ateKeyframesRMSE = 0;
        int ateAllframesN = 0, ateKeyframesN = 0;

        for (int i = 0; i < cameras.size(); i++)
        {
            if (align && groundtruth[i].has_value()) {
                ateAllframesRMSE += (cameras[i].eye() - groundtruth[i].value().eye()).squaredNorm();
                if (iskeyframe[i]) {
                    ateKeyframesRMSE += (cameras[i].eye() - groundtruth[i].value().eye()).squaredNorm();
                    ateKeyframesN++;
                }
                ateAllframesN++;
            }

            Matrix34 pose = cameras[i].nullspaceCameraMatrix34();

            bool isFirst = true;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 4; j++) {
                    if (isFirst) {
                        isFirst = false;
                    } else {
                        file << " ";
                    }
                    file << pose(i, j);
                }
            }

            file << "\n";

        }

        file.close();

        if (ateAllframesN > 0) {
            ateAllframesRMSE = ateAllframesRMSE / (scalar_t)ateAllframesN;
            ateAllframesRMSE = sqrt(ateAllframesRMSE);
            logger.info("ATE RMSE : " + std::to_string(ateAllframesRMSE));
        }

        if (ateKeyframesN > 0) {
            ateKeyframesRMSE = ateKeyframesRMSE / (scalar_t)ateKeyframesN;
            ateKeyframesRMSE = sqrt(ateKeyframesRMSE);
            logger.info("ATE Key RMSE : " + std::to_string(ateKeyframesRMSE));
        }

    }

    logger.important("Writing finished");


}

bool CML::Map::mergeMapPoints(PPoint mapPointA, PPoint mapPointB) {
    if (mapPointA == mapPointB) {
        logger.debug("You tried to merge the same map point");
        return true;
    }
    if (!mapPointA->isGroup(0)) {
        logger.fatal("merge map point with a point which belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return false;
    }
    if (!mapPointB->isGroup(0)) {
        logger.fatal("merge map point with a point which belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return false;
    }

    // Store the direct apparitions of B
    auto directApparitions = mapPointB->getDirectApparitions();

    // Store the indirect apparitions of B
    List<Pair<PFrame, FeatureIndex>> indirectApparitions;
    for (auto frame : mapPointB->getIndirectApparitions()) {
        FeatureIndex index = frame->getIndex(mapPointB);
        assertThrow(index.hasValidValue(), "Some strange bug");
        indirectApparitions.emplace_back(frame, index);
    }

    // For each frame in the indirect apparitions of B, check that A is not already inside in a another index
    for (auto [frame, index] : indirectApparitions) {
        if (frame->getIndex(mapPointA).hasValidValue()) {
            logger.debug("Can't merge map points because of a conflict");
            return false;
        }
    }

    removeMapPoint(mapPointB);
    for (auto [frame, index] : indirectApparitions) {
        bool res = frame->setMapPoint(index, mapPointA);
        assertThrow(res, "Set map point failed");
    }
    for (auto frame : directApparitions) {
        frame->addDirectApparitions(mapPointA);
    }
    logger.debug("Merged two map points");
    return true;
}

bool CML::Map::canMargePoints(PPoint mapPointA, PPoint mapPointB) {
    if (mapPointA == mapPointB) {
        logger.debug("You tried to merge the same map point");
        return false;
    }
    List<Pair<PFrame, FeatureIndex>> apparitions;
    for (auto frame : mapPointB->getIndirectApparitions()) {
        FeatureIndex index = frame->getIndex(mapPointB);
        apparitions.emplace_back(frame, index);
    }
    for (auto [frame, index] : apparitions) {
        OptPPoint mapPoint = frame->getMapPoint(index);
        if (mapPoint.isNotNull() && mapPointA != mapPoint && mapPointB != mapPoint) {
            return false;
        }
    }
    return true;
}