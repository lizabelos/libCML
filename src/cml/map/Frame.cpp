//
// Created by tbelos on 04/06/19.
//

#include "cml/map/Frame.h"
#include "cml/map/Map.h"
#include "cml/utils/KDTree.h"
#include "cml/maths/Utils.h"

CML::PoolAllocator CML::Frame::allocator{1024};

void CML::Frame::setCamera(const Camera &camera, bool updateDeforms) {

    assertDeterministic("Set Camera Translation", camera.getTranslation().sum());
    assertDeterministic("Set Camera Translation", camera.getQuaternion().getParameters().sum());

    mCamera = camera;

    Vector3 center = mCamera.eye();
    mCameraCenter[0] = center[0];
    mCameraCenter[1] = center[1];
    mCameraCenter[2] = center[2];

    if (updateDeforms) {
        LockGuard lg(mDeformsMutex);
        for (auto deform : mDeforms) {
            deform.camera = deform.frame0->getCamera().to(getCamera());
            deform.camera = Camera(deform.camera.getTranslation() / (deform.frame0->getCamera().eye() - deform.frame1->getCamera().eye()).norm(), deform.camera.getQuaternion());
        }
    }

    for (Observer *observer : mObservers) {
        observer->onCameraChanged(this, camera);
    }

    {
        LockGuard lg(mMapPointsMutex);
        for (auto[point, index] : mMapPointsIndex) {
            point->onCameraChange(camera);
        }
    }

    {
        LockGuard lg(mMapPointsApparitionsMutex);
        for (auto point : mMapPointsApparitions) {
            point->onCameraChange(camera);
        }
    }
}

void CML::Frame::setCameraAndDeform(const HashMap<PFrame, Camera> &frames) {
    Set<PFrame> skip;
    setCameraAndDeform(frames, skip);
}


void CML::Frame::setCameraAndDeform(const HashMap<PFrame, Camera> &frames, Set<PFrame> &skip) {

    bool isFirst = skip.size() == 0;

    if (frames.size() == 0) {
        return;
    }

    Set<PFrame> framesToDeform;
    HashMap<PFrame, Camera> newFramesAndCamera;

    for (auto [frame, camera] : frames) {

        frame->setCamera(camera, isFirst);
        skip.insert(frame);

        LockGuard lg(frame->mToDeformMuex);
        for (auto frameToDeform : frame->mToDeform) {
            if (skip.find(frameToDeform) != skip.end()) {
                continue;
            }
            framesToDeform.insert(frameToDeform);
        }

    }

    return;

    for (auto frame : framesToDeform) {
        newFramesAndCamera[frame] = frame->computeNewCameraFromDeforms();
    }

    setCameraAndDeform(newFramesAndCamera, skip);

}


void CML::Frame::setCameraWithoutObserver(const Camera &camera) {
    mCamera = camera;

    Vector3 center = mCamera.eye();
    mCameraCenter[0] = center[0];
    mCameraCenter[1] = center[1];
    mCameraCenter[2] = center[2];

    {
        LockGuard lg(mDeformsMutex);
        for (auto deform : mDeforms) {
            deform.camera = deform.frame0->getCamera().to(getCamera());
            deform.camera = Camera(deform.camera.getTranslation() / (deform.frame0->getCamera().eye() - deform.frame1->getCamera().eye()).norm(), deform.camera.getQuaternion());
        }
    }

    {
        LockGuard lg(mMapPointsMutex);
        for (auto[point, index] : mMapPointsIndex) {
            point->onCameraChange(camera);
        }
    }

    {
        LockGuard lg(mMapPointsApparitionsMutex);
        for (auto point : mMapPointsApparitions) {
            point->onCameraChange(camera);
        }
    }
}

void CML::Frame::processNearestNeighbors(int group, Vector2 position, int num, List<NearestNeighbor> &result) const {
    std::shared_ptr<PointGrid<Corner>> pointKdTree;

    {
        LockGuard lg(mFeatureMutex);
        pointKdTree = mFeaturePointTree[group];
    }

    pointKdTree->searchInRadiusNum(position, num, result);
}

void CML::Frame::processNearestNeighborsInRadius(int group, Vector2 position, float distance, List<NearestNeighbor> &result) const {

    std::shared_ptr<PointGrid<Corner>> pointKdTree;

    {
        LockGuard lg(mFeatureMutex);
        pointKdTree = mFeaturePointTree[group];
    }

    pointKdTree->searchInRadius(position, distance, result);

}

int CML::Frame::addFeaturePoints(const List<Corner> &features, Ptr<Features::BoW, Nullable> bow) {

    assertThrow(features.size() > 0, "Empty features list");

    int group;
    {
        LockGuard lg(mFeatureMutex);
        LockGuard lg2(mMapPointsMutex);
        group = mFeaturePoints.size();
        mFeaturePoints.emplace_back(List<Corner>());
        mFeaturePointTree.emplace_back(std::shared_ptr<PointGrid<Corner>>());
        mBoW.emplace_back(bow);
        mMapPoints.emplace_back(List<OptPPoint>(features.size()));
    }

    std::copy(features.begin(), features.end(), std::back_inserter(mFeaturePoints[group]));
    mFeaturePointTree[group] = std::make_shared<PointGrid<Corner>>(mFeaturePoints[group], Vector2i(0,0), Vector2i(getWidth(0), getHeight(0)));

    return group;

}

bool CML::Frame::setMapPoint(FeatureIndex i, OptPPoint mapPoint) {

    LockGuard lg(mMapPointsMutex);

    if (mapPoint.isNull()) {
        logger.fatal("set map point with a null point. ( continuing anyway, this may be a recently remove point )");
        return false;
    }

    if (!mapPoint->isGroup(0)) {
        logger.fatal("set map point with a point which not belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return false;
    }

    if (mMapPoints[i.group][i.index].isNotNull()) {
        if (mMapPoints[i.group][i.index] == mapPoint) {
            logger.debug("setMapPoint twice (consistent result)");
            return true;
        } else {
            logger.debug("setMapPoint twice (same index, result not consistent)");
            return false;
        }
    }

    if (mMapPointsIndex.count(mapPoint) > 0) {
        if (mMapPointsIndex[mapPoint] == i) {
            logger.debug("setMapPoint twice (consistent result)");
            return true;
        } else {
            logger.debug("setMapPoint twice (same MapPoint, result not consistent)");
            return false;
        }
    }

    mMapPoints[i.group][i.index] = mapPoint;
    mMapPointsIndex.insert(Pair<PPoint, FeatureIndex>(mapPoint, i));
    mOrderedMapPoints.insert(mapPoint);

    Corner corner;
    {
        LockGuard lg(mFeatureMutex);
        corner = mFeaturePoints[i.group][i.index];
    }
    mapPoint->addIndirectApparition(this, corner);
    // mapPoint->subscribeObserver(this);

    return true;
}

CML::Camera CML::Frame::getEvaluationPosition() {
    return mCamera;
}

/*
CML::scalar_t CML::Frame::processSceneMedianDepth(const int q) {
    List<scalar_t> depths;
    for (auto [index, point] : getMapPoints()) {
        if (point->isGroup(MAPPED)) {
            depths.emplace_back(point->getWorldCoordinate().relative(getCamera()).z());
        }
    }

    std::sort(depths.begin(),depths.end());

    return depths[(depths.size()-1)/q];
}*/

void CML::Frame::removeMapPoint(PPoint mapPoint) {

    LockGuard lg1(mMapPointsMutex);
    LockGuard lg2(mMapPointsApparitionsMutex);

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        onMapPointGroupChange(mapPoint, i, false);
    }

    auto indexSearch = mMapPointsIndex.find(mapPoint);
    if (indexSearch != mMapPointsIndex.end()) {
        FeatureIndex index = indexSearch->second;
        mMapPoints[index.group][index.index] = nullptr;
        mMapPointsIndex.erase(indexSearch);
        mOrderedMapPoints.erase(mapPoint);
        mapPoint->removeIndirectApparition(this);
    }

    auto apparitionSearch = mMapPointsApparitions.find(mapPoint);
    if (apparitionSearch != mMapPointsApparitions.end()) {
        mMapPointsApparitions.erase(apparitionSearch);
        mapPoint->removeDirectApparition(this);
    }

}

void CML::Frame::onMapPointErased(PPoint mapPoint) {
    LockGuard lg1(mMapPointsMutex);
    LockGuard lg2(mMapPointsApparitionsMutex);

    auto indexSearch = mMapPointsIndex.find(mapPoint);
    if (indexSearch != mMapPointsIndex.end()) {
        FeatureIndex index = indexSearch->second;
        mMapPoints[index.group][index.index] = nullptr;
        mMapPointsIndex.erase(indexSearch);
        mOrderedMapPoints.erase(mapPoint);
    }

    auto apparitionSearch = mMapPointsApparitions.find(mapPoint);
    if (apparitionSearch != mMapPointsApparitions.end()) {
        mMapPointsApparitions.erase(mapPoint);
    }
}

void CML::Frame::addDirectApparitions(OptPPoint mapPoint) {
    LockGuard lg(mMapPointsApparitionsMutex);

    if (mapPoint.isNull()) {
        logger.fatal("add direct apparitions with a null point. ( continuing anyway, this may be a recently remove point )");
        return;
    }

    if (!mapPoint->isGroup(0)) {
        logger.fatal("add direct apparitions a point which not belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return;
    }

    if (mMapPointsApparitions.find(mapPoint) == mMapPointsApparitions.end()) {
        mMapPointsApparitions.insert(mapPoint);
        mapPoint->addDirectApparition(this);
        // mapPoint->subscribeObserver(this);
    }
}

void CML::Frame::onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) {

    assertThrow(groupId >= 0 && groupId < MAPOBJECT_GROUP_MAXSIZE, "Max group out of range");

    {
        LockGuard lg(mGroupsMapPointMutexes[groupId]);
        if (state) {
            mGroupsMapPoint[groupId].insert(mapPoint);
        } else {
            mGroupsMapPoint[groupId].erase(mapPoint);
        }
    }

    if (mapPoint->getReferenceFrame() == this) {

        LockGuard lg(mGroupsMapPointReferenceMutexes[groupId]);
        if (state) {
            mGroupsMapPointReference[groupId].insert(mapPoint);
        } else {
            mGroupsMapPointReference[groupId].erase(mapPoint);
        }

    }

}

void CML::Frame::addDeformSingleDirection(PFrame frame0, PFrame frame1) {
    assertThrow(frame0 != this && frame1 != this, "Can't add a deformation with myself");
    assertThrow(frame0 != frame1, "Can't add a deformation with the same frame");
    Deform deform(frame0, frame1);
    deform.camera = frame0->getCamera().to(getCamera());
    deform.camera = Camera(deform.camera.getTranslation() / (frame0->getCamera().eye() - frame1->getCamera().eye()).norm(), deform.camera.getQuaternion());
    addDeform(deform);
}

CML::Camera CML::Frame::computeNewCameraFromDeforms() {

    LockGuard lg(mDeformsMutex);

    if (mDeforms.size() == 0) {
        return mCamera;
    }

    Vector3 translation = Vector3::Zero();
    Vector3 axisAngle = Vector3::Zero();

    for (auto deform : mDeforms) {

        Camera camera(deform.camera.getTranslation() * (deform.frame0->getCamera().eye() - deform.frame1->getCamera().eye()).norm(), deform.camera.getQuaternion());
        camera = deform.frame0->getCamera().compose(camera);

        translation = translation + camera.getTranslation();
        axisAngle = axisAngle + camera.getAxisAngle().getParameters();

    }

    translation = translation / (float)mDeforms.size();
    axisAngle = axisAngle / (float)mDeforms.size();

    return Camera(translation, mCamera.getQuaternion());

}

int CML::Frame::shared(int groupId, PFrame other) {
    int numCommon;

    auto points = getGroupMapPoints(groupId);
    for (PPoint point : points) {
        if (point->haveIndirectApparition(other)) {
            numCommon++;
        }
    }

    return numCommon;
}

CML::scalar_t CML::Frame::computeMedianDepth(bool useDirect, bool useIndirect) {
    List<scalar_t> allDepths;
    allDepths.reserve(10000);

    if (useDirect) {
        for (auto point : getMapPointsApparitions()) {
            allDepths.emplace_back(point->getWorldCoordinate().relative(getCamera()).z());
        }
    }
    if (useIndirect) {
        for (auto[index, point] : getMapPoints()) {
            allDepths.emplace_back(point->getWorldCoordinate().relative(getCamera()).z());
        }
    }
    if (allDepths.empty()) {
        return 0;
    }
    return median(allDepths);
}
