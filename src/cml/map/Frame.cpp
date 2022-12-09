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

void CML::Frame::setCameraAndDeform(const FrameHashMap<Camera> &frames) {
    FrameSet skip;
    setCameraAndDeform(frames, skip);
}


void CML::Frame::setCameraAndDeform(const FrameHashMap<Camera> &frames, FrameSet &skip) {

    bool isFirst = skip.size() == 0;

    if (frames.size() == 0) {
        return;
    }

    FrameSet framesToDeform;
    FrameHashMap<Camera> newFramesAndCamera;

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
    signalMethodStart("Frame::processNearestNeighbors");
    std::shared_ptr<PointGrid<Corner>> pointKdTree;

    {
        LockGuard lg(mFeatureMutex);
        pointKdTree = mFeaturePointTree[group];
    }

    pointKdTree->searchInRadiusNum(position, num, result);

}

void CML::Frame::processNearestNeighborsInRadius(int group, Vector2 position, float distance, List<NearestNeighbor> &result) const {
    signalMethodStart("Frame::processNearestNeighborsInRadius");

    std::shared_ptr<PointGrid<Corner>> pointKdTree;

    {
        LockGuard lg(mFeatureMutex);
        pointKdTree = mFeaturePointTree[group];
    }

    pointKdTree->searchInRadius(position, distance, result);


}

int CML::Frame::addFeaturePoints(const List<Corner> &features, Ptr<Features::BoW, Nullable> bow) {
    signalMethodStart("Frame::addFeaturePoints");

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
    signalMethodStart("Frame::setMapPoint");

    LockGuard lg(mMapPointsMutex);

    if (mapPoint.isNull()) {
        CML_LOG_FATAL("set map point with a null point. ( continuing anyway, this may be a recently remove point )");
        return false;
    }

    if (!mapPoint->isGroup(0)) {
        CML_LOG_FATAL("set map point with a point which not belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return false;
    }

    if (mMapPoints[i.group][i.index].isNotNull()) {
        if (mMapPoints[i.group][i.index] == mapPoint) {
            CML_LOG_DEBUG("setMapPoint twice (consistent result)");
            return true;
        } else {
            CML_LOG_DEBUG("setMapPoint twice (same index, result not consistent)");
            return false;
        }
    }

    if (mMapPointsIndex.count(mapPoint) > 0) {
        if (mMapPointsIndex[mapPoint] == i) {
            CML_LOG_DEBUG("setMapPoint twice (consistent result)");
            return true;
        } else {
            CML_LOG_DEBUG("setMapPoint twice (same MapPoint, result not consistent)");
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
    signalMethodStart("Frame::removeMapPoint");

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
    signalMethodStart("Frame::onMapPointErased");
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
    signalMethodStart("Frame::addDirectApparitions");
    LockGuard lg(mMapPointsApparitionsMutex);

    if (mapPoint.isNull()) {
        CML_LOG_FATAL("add direct apparitions with a null point. ( continuing anyway, this may be a recently remove point )");
        return;
    }

    if (!mapPoint->isGroup(0)) {
        CML_LOG_FATAL("add direct apparitions a point which not belong to mapped group. ( continuing anyway, this may be a recently remove point )");
        return;
    }

    if (mMapPointsApparitions.find(mapPoint) == mMapPointsApparitions.end()) {
        mMapPointsApparitions.insert(mapPoint);
        mapPoint->addDirectApparition(this);
        // mapPoint->subscribeObserver(this);
    }

}

void CML::Frame::onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) {
    //signalMethodStart("Frame::onMapPointGroupChange");

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
    signalMethodStart("Frame::shared");
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
    signalMethodStart("Frame::computeMedianDepth");
    MedianComputer<scalar_t> medianComputer;

    if (useDirect) {
        LockGuard lg(mMapPointsApparitionsMutex);
        for (auto point : mMapPointsApparitions) {
            medianComputer.addValue(point->getWorldCoordinate().relative(getCamera()).z());
        }
    }
    if (useIndirect) {
        LockGuard lg(mMapPointsMutex);
        for (size_t groupId = 0; groupId < mMapPoints.size(); groupId++) {
            for (size_t index = 0; index < mMapPoints[groupId].size(); index++) {
                if (mMapPoints[groupId][index].isNotNull()) {
                    medianComputer.addValue(mMapPoints[groupId][index]->getWorldCoordinate().relative(getCamera()).z());
                }
            }
        }
    }
    if (!medianComputer.isInitialized()) {
        return 0;
    }
    scalar_t result = medianComputer.getMedian();

    return result;
}

namespace CML {
    float sign(Vector2 p1, Vector2 p2, Vector2 p3) {
        return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
    }

    bool pointInTriangle(Vector2 pt, Vector2 v1, Vector2 v2, Vector2 v3) {
        float d1, d2, d3;
        bool has_neg, has_pos;

        d1 = sign(pt, v1, v2);
        d2 = sign(pt, v2, v3);
        d3 = sign(pt, v3, v1);

        has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    template<typename T> T min(T&&t)
    {
        return std::forward<T>(t);
    }

    template<typename T0, typename T1, typename... Ts> typename std::common_type<T0, T1, Ts...>::type min(T0&& val1, T1&& val2, Ts&&... vs)
    {
        if (val2 < val1) return min(val2, std::forward<Ts>(vs)...);
        else return min(val1, std::forward<Ts>(vs)...);
    }

    template<typename T> T max(T&&t)
    {
        return std::forward<T>(t);
    }

    template<typename T0, typename T1, typename... Ts> typename std::common_type<T0, T1, Ts...>::type max(T0&& val1, T1&& val2, Ts&&... vs)
    {
        if (val2 > val1) return max(val2, std::forward<Ts>(vs)...);
        else return max(val1, std::forward<Ts>(vs)...);
    }


    void triangle(Array2D<Vector2> &depthMap, Image &image, Vector3 a, ColorRGBA ac, Vector3 b, ColorRGBA bc, Vector3 c, ColorRGBA cc) {

        int minX = min(a(0), b(0), c(0)) - 1;
        int minY = min(a(1), b(1), c(1)) - 1;
        int maxX = max(a(0), b(0), c(0)) + 1;
        int maxY = max(a(1), b(1), c(1)) + 1;

        minX = max(minX, 0);
        minY = max(minY, 0);
        maxX = min(maxX, image.getWidth());
        maxY = min(maxY, image.getHeight());

        for (int y = minY; y < maxY; y++) {

            for (int x = minX; x < maxX; x++) {

                Vector2 p(x, y);

                if (pointInTriangle(p, a.head<2>(), b.head<2>(), c.head<2>())) {

                    float weightA = (p - a.head<2>()).norm();
                    float weightB = (p - b.head<2>()).norm();
                    float weightC = (p - c.head<2>()).norm();
                    float weightTotal = weightA + weightB + weightC;
                    weightA /= weightTotal;
                    weightB /= weightTotal;
                    weightC /= weightTotal;

                    float depth = a(2) * weightA + b(2) * weightB + c(2) * weightC; // interpolate depth;
                    Vector3f colorE = ac.eigen().cast<float>() * weightA + bc.eigen().cast<float>() * weightB + cc.eigen().cast<float>() * weightC; // interpolate color
                    ColorRGBA color(colorE.x(), colorE.y(), colorE.z());

                    int depthMapX = x * depthMap.getWidth() / image.getWidth();
                    int depthMapY = y * depthMap.getHeight() / image.getHeight();

                    if (depth <= 0) {
                        //image(x, y) = ColorRGBA(0,0,255,255);
                    } else if (!std::isfinite(depthMap(depthMapX, depthMapY)(0)) || depthMap(depthMapX, depthMapY)(0) > depth) {
                        depthMap(depthMapX, depthMapY)(0) = depth;
                        image(x, y) = color;
                    } else {
                        //image(x, y) = ColorRGBA(255,0,0,255);
                    }

                }

            }

        }

    }


    void loadObj(const std::string &path, List<Vector3> &triangles, List<ColorRGBA> &colors) {

        // load obj file
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cout << "Error: could not open file " << path << std::endl;
            return;
        }

        List<Vector3> vertices;
        List<List<int>> faces;
        List<ColorRGBA> verticesColors;

        std::string line;
        while (std::getline(file, line)) {
            if (line.substr(0, 2) == "v ") {
                std::istringstream s(line.substr(2));
                Vector3 v; s >> v(0); s >> v(1); s >> v(2);
                v.y() = -v.y();
                vertices.push_back(v);
            }
            else if (line.substr(0, 2) == "f ") {
                // Split the face to face id/texture id/normal id
                std::vector<std::string> splitted;
                List<int> face;
                split(line, splitted, ' ');
                for (int j = 1; j < splitted.size(); j++) {
                    std::vector<std::string> splitted2;
                    split(splitted[j], splitted2, '/');
                    face.emplace_back(std::stoi(splitted2[0]));
                }
                faces.emplace_back(face);
            }
            else if (line.substr(0, 2) == "c ") {
                std::istringstream s(line.substr(2));
                ColorRGBA c;
                s >> c(0); s >> c(1); s >> c(2); s >> c(3);
                verticesColors.push_back(c);
            }
        }

        // convert to triangles
         float factor = 0.2;
        Vector3 offset(-2,-1,7.0);
        for (size_t i = 0; i < faces.size(); i++) {
            if (faces[i].size() == 3) {
                triangles.push_back(vertices[faces[i][0] - 1]  * factor + offset);
                triangles.push_back(vertices[faces[i][1] - 1]  * factor + offset);
                triangles.push_back(vertices[faces[i][2] - 1]  * factor + offset);
                if (verticesColors.size() > 0) {
                    colors.push_back(verticesColors[faces[i][0] - 1]);
                    colors.push_back(verticesColors[faces[i][1] - 1]);
                    colors.push_back(verticesColors[faces[i][2] - 1]);
                } else {
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                }
            }
            else if (faces[i].size() == 4) {
                triangles.push_back(vertices[faces[i][0] - 1] * factor + offset);
                triangles.push_back(vertices[faces[i][1] - 1] * factor + offset);
                triangles.push_back(vertices[faces[i][2] - 1] * factor + offset);
                triangles.push_back(vertices[faces[i][1] - 1] * factor + offset);
                triangles.push_back(vertices[faces[i][2] - 1] * factor + offset);
                triangles.push_back(vertices[faces[i][3] - 1] * factor + offset);
                if (verticesColors.size() > 0) {
                    colors.push_back(verticesColors[faces[i][0] - 1]);
                    colors.push_back(verticesColors[faces[i][1] - 1]);
                    colors.push_back(verticesColors[faces[i][2] - 1]);
                    colors.push_back(verticesColors[faces[i][1] - 1]);
                    colors.push_back(verticesColors[faces[i][2] - 1]);
                    colors.push_back(verticesColors[faces[i][3] - 1]);
                } else {
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                    colors.push_back(ColorRGBA(128));
                }
            }
            else {
                std::cout << "Error: face " << i << " has " << faces[i].size() << " vertices" << std::endl;
            }
        }

    }

}


void CML::Frame::computeVirtualRealityImage(const PointSet &points) {

    if (getId() % 10 != 0) {
        return;
    }

    CML_LOG_IMPORTANT("Computing virtual reality image");

    int levelForDepthMap = 2;

    Array2D<Vector2> depthMap(getWidth(levelForDepthMap), getHeight(levelForDepthMap), Vector2(0, 0));
    Array2D<bool> depthMapProjected(getWidth(levelForDepthMap), getHeight(levelForDepthMap), false);

    int removeBecauseOfUncertainty = 0, removedBecauseNotInside = 0;
    for (auto point : points) {
        if (point->getUncertainty() > 1) {
            removeBecauseOfUncertainty++;
            continue;
        }
        DistortedVector2d p2d = distort(point->getWorldCoordinate().project(getCamera()), levelForDepthMap);
        Vector3 rposition = point->getWorldCoordinate().relative(getCamera());
        if (rposition(2) < 0) {
            continue;
        }
        if (isInside(p2d, levelForDepthMap, 0)) {
            depthMap(p2d.x(), p2d.y()) += Vector2(rposition(2) / point->getUncertainty(), 1.0f / point->getUncertainty());
            depthMapProjected(p2d.x(), p2d.y()) = true;
        } else {
            removedBecauseNotInside++;
        }
    }

    CML_LOG_IMPORTANT("For " + std::to_string(points.size()) + " points, " + std::to_string(removeBecauseOfUncertainty) + " have been removed for uncertainty, and " + std::to_string(removedBecauseNotInside) + " have been remove because not inside");



    List<Vector2> nns;
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            if (x != 0 || y != 0) {
                nns.emplace_back(x, y);
            }
        }
    }

    const float colorThreshold = 50;

    // Your are modifying the depth map with the modified depth map shiiiiiiiiiiit
   for (int i = 0; i < 1000; i++) {

       Array2D<Vector2> tmpDepthMap = depthMap;

       for (int y = 1; y < depthMap.getHeight() - 1; y++) {
            for (int x = 1; x < depthMap.getWidth() - 1; x++) {
                for (auto nn : nns) {

                    Vector2i c(x, y);
                    Vector2i p = c + nn.cast<int>();


                    //depthMap(p) += depthMap(c) * 0.1f;

                    float colorDiff = abs(getCaptureFrame().getGrayImage(levelForDepthMap).get(c.x(), c.y()) - getCaptureFrame().getGrayImage(levelForDepthMap).get(p.x(), p.y())) + 1;

                    float weight = 0.001f / colorDiff;
                    tmpDepthMap(p) += Vector2(depthMap(c) * weight);

                }


            }

        }

       depthMap = tmpDepthMap;

   }


   List<float> allDepths;
    for (int y = 0; y < depthMap.getHeight(); y++) {

        for (int x = 0; x < depthMap.getWidth(); x++) {

            if (depthMap(x, y)(1) == 0 || depthMap(x, y)(0) == 0) {
                depthMap(x, y)(0) = 100000;
                continue;
            }

            depthMap(x, y) /= depthMap(x, y)(1); // we allow infinite. infinite = depth 0
            //depthMap(x, y)(0) = 1.0f / depthMap(x, y)(0); // from idepth to depth
            if (std::isfinite( depthMap(x, y)(0))) {
                allDepths.emplace_back(depthMap(x, y)(0));
            } else {
                CML_LOG_ERROR("NON FINITE");
            }
        }

    }

    List<Vector3> triangles;
    List<ColorRGBA> colors;

    loadObj("resources/80-pikachu/Pikachu OBJ.obj", triangles, colors);

    auto colorTransition = Exposure().to(getExposure());
    for (int i = 0; i < colors.size(); i++) {
        colors[i] = ColorRGBA(colorTransition(colors[i].r()), colorTransition(colors[i].g()), colorTransition(colors[i].b()));
    }

    List<Vector3> projections;
    projections.resize(triangles.size());

    for (int i = 0; i < projections.size(); i++) {
        projections[i].head<2>() = distort(WorldPoint::fromAbsolute(triangles[i]).project(getCamera()), 0);
        projections[i](2) = WorldPoint::fromAbsolute(triangles[i]).relative(getCamera()).z();
    }

    CML_LOG_IMPORTANT("Retrieving color image");


    Image tmpImage = getCaptureFrame().getGrayImage(0).cast<ColorRGBA>();
  //  tmpImage.horizontalFlip().saveBmp("z_undistortred" + std::to_string(getId()) + ".bmp");

    CML_LOG_IMPORTANT("Drawing");

    for (int i = 0; i < projections.size(); i = i + 3) {
        triangle(depthMap, tmpImage, projections[i], colors[i], projections[i+1], colors[i+1], projections[i+2], colors[i+2]);
    }


    CML_LOG_IMPORTANT("Saving image");

   // tmpImage.horizontalFlip().saveBmp("z_vr_" + std::to_string(getId()) + ".bmp");

    if (allDepths.size() > 0) {
        float med = median(allDepths);

        CML_LOG_IMPORTANT("Median depth : " + std::to_string(med));

        for (int y = 0; y < depthMap.getHeight(); y++) {

            for (int x = 0; x < depthMap.getWidth(); x++) {

                float color = depthMap(x, y)(0) * 255 / (med * 2.0f);
                if (color > 255) color = 255;
                if (color < 0) color = 0;
                tmpImage(x, y) = ColorRGBA(color);
            }

        }

        tmpImage.horizontalFlip().saveBmp("z_depthmap_" + std::to_string(getId()) + ".bmp");
    }



}