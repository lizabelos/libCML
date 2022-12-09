#include "cml/optimization/g2o/IndirectBundleAdjustment.h"
#include "cml/optimization/g2o/g2oconfig.h"
#include "cml/utils/Complexity.h"

#include "cml/maths/Utils.h"

bool CML::Optimization::G2O::IndirectBundleAdjustment::localOptimize(PFrame currentFrame, int frameGroup, bool *pbStopFlag, bool fixFrames) {

    // Local KeyFrames: First Breath Search from Current Keyframe
    lLocalKeyFrames.clear();
    lFixedCameras.clear();
    lLocalIndirectPoints.clear();

    lLocalKeyFrames.insert(currentFrame);
    for (auto frame : getMap().processIndirectCovisiblity(currentFrame, mIndirectCovisiblityMax.i(), frameGroup)) {
        lLocalKeyFrames.insert(frame);
    }

    // Local MapPoints seen in Local KeyFrames
    for (auto pKFi : lLocalKeyFrames) {
        for (auto [index, pMP] : pKFi->getMapPoints()) {
            if (pMP->isGroup(getMap().INDIRECTGROUP) && pMP->getIndirectApparitionNumber() > 1) {
                lLocalIndirectPoints.insert(pMP);
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    for (auto point : lLocalIndirectPoints) {
        for (auto frame : point->getIndirectApparitions()) {
            if (frame->isGroup(frameGroup) && lLocalKeyFrames.count(frame) == 0) {
                lFixedCameras.insert(frame);
            }
        }
    }

    if (lLocalIndirectPoints.empty()) {
        CML_LOG_ERROR("G2O BA : No points");
        return false;
    }


    if (lLocalKeyFrames.size() <= 2) {
        CML_LOG_ERROR("G2O BA : Not enough frames");
        return false;
    }

    if (mOptimizer != nullptr) {
        delete mOptimizer;
    }
    mOptimizer = new g2o::SparseOptimizer;
    
    g2o::OptimizationAlgorithm* solver;
    if (!fixFrames) {
        solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<g2o::BlockSolver_6_3>(
                        g2o::make_unique<DefaultG2OSolverForSpeed<g2o::BlockSolver_6_3::PoseMatrixType>>())
        );
    } else {
        solver = new DefaultG2OSolverForStructureOnly();
    }

    mOptimizer->setAlgorithm(solver);

    if(pbStopFlag != nullptr) {
        mOptimizer->setForceStopFlag(pbStopFlag);
    }

    maxKFid = 0;

    // Set Local KeyFrame vertices
    for (auto frame : lLocalKeyFrames)
    {
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(frame->getCamera().getRotationMatrix().cast<number_t>(), frame->getCamera().getTranslation().cast<number_t>()));
        vSE3->setId(frame->getId() + idOffset);
        vSE3->setFixed(fixFrames);
        mOptimizer->addVertex(vSE3);
        if(frame->getId() > maxKFid) {
            maxKFid = frame->getId() + idOffset;
        }
    }

    // Set Fixed KeyFrame vertices
    for(auto frame : lFixedCameras)
    {
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(g2o::SE3Quat(frame->getCamera().getRotationMatrix().cast<number_t>(), frame->getCamera().getTranslation().cast<number_t>()));
        vSE3->setId(frame->getId() + idOffset);
        vSE3->setFixed(true);
        mOptimizer->addVertex(vSE3);
        if(frame->getId() > maxKFid) {
            maxKFid = frame->getId() + idOffset;
        }
    }

    if (lFixedCameras.size() < 3) {
        CML_LOG_ERROR("G2O BA : Not enough fixed cameras");
        return false;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size()) * lLocalIndirectPoints.size();

    vpEdges.clear();
    vpEdgesPairs.clear();
    vpEdges.reserve(nExpectedSize);
    vpEdgesPairs.reserve(nExpectedSize);

    const float thHuberIndirect = sqrt(5.991);

    int numIndirect = 0;



    for(auto point : lLocalIndirectPoints)
    {
        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(point->getWorldCoordinate().absolute().cast<number_t>());
        int id = point->getId() + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        bool r = mOptimizer->addVertex(vPoint);
        assertThrow(r, "Vertex already exist");

        //Set edges
        for (auto frame : point->getIndirectApparitions())
        {

            if(lLocalKeyFrames.count(frame) > 0 || lFixedCameras.count(frame) > 0)
            {

                auto corner = frame->getFeaturePoint(point).value();

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(mOptimizer->vertex(frame->getId() + idOffset)));
                e->setMeasurement(corner.point0().cast<number_t>());

                scalar_t scaleFactor = corner.processScaleFactorFromLevel();
                scalar_t invSigma2 = 1.0 / (scaleFactor * scaleFactor);
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberIndirect);

                Matrix33 K = frame->getK(0);
                e->fx = K(0,0);
                e->fy = K(1,1);
                e->cx = K(0, 2);
                e->cy = K(1, 2);

                mOptimizer->addEdge(e);
                vpEdges.push_back(e);
                assertThrow(point->isGroup(getMap().DIRECTGROUP) == false, "whaaaat ??");

                vpEdgesPairs.emplace_back(frame, point);

                numIndirect++;

            }
        }

    }

    if (numIndirect < 10) {
        CML_LOG_ERROR("G2O Ba : Not enough indirect points");
        return false;
    }

    if(pbStopFlag != nullptr) {
        if (*pbStopFlag) {
            CML_LOG_ERROR("G2O Ba: Stop flag is set to true");
            return false;
        }
    }

    if (!mOptimizer->initializeOptimization()) {
        CML_LOG_ERROR("G2O Initialize optimization failed");
        return false;
    }

    CML_LOG_INFO("G2O is optimizing");
    mTimer.start();
    startOptimization(mNumIteration.i(), true, false);

    if (mRefineIteration.i() > 0) {

        CML_LOG_IMPORTANT("Refining : " + std::to_string(mRefineIteration.i()));

        if (pbStopFlag != nullptr) {
            if (*pbStopFlag) {
                CML_LOG_ERROR("G2O Ba: Stop flag is set to true");
                return true;
            }
        }

        startOptimization(mRefineIteration.i(), true, true);

    } else {
        CML_LOG_IMPORTANT("Not refining : " + std::to_string(mRefineIteration.i()));

    }

    return true;


}

void CML::Optimization::G2O::IndirectBundleAdjustment::startOptimization(int num, bool enableDropout, bool onlyRobust) {
    signalMethodStart("G2O::IndirectBundleAdjustment::startOptimization");

    if (mDropout.f() <= 0) {
        enableDropout = false;
    }

    for (size_t i = 0, iend = vpEdges.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ *e = vpEdges[i];
        e->setLevel(0);
        // todo : set kernel (memory problem ?)
    }

    if (onlyRobust) {
        for (size_t i = 0, iend = vpEdges.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZ *e = vpEdges[i];
            e->setLevel(0);

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(nullptr);

        }
    }

    mOptimizer->initializeOptimization(0);
    mOptimizer->optimize(num);
/*
    if (!enableDropout) {
        mOptimizer->initializeOptimization(0);
    }

    std::random_device dev;
    std::uniform_real_distribution<> dist(0.0, 1.0);

    for (int it = 0; it < num; it++) {

        if (enableDropout) for (size_t i = 0, iend = vpEdges.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZ *e = vpEdges[i];
            if (e->level() != 1) {
                double v = dist(dev);
                if (v < mDropout.f()) {
                    e->setLevel(2);
                } else {
                    e->setLevel(0);
                }
            }
        }

        mTimer.stop();
        if (mTimer.getValue() > mTimeLimit.f()) {
            CML_LOG_IMPORTANT("IBA time limit reached");
            return;
        }

        CML_LOG_IMPORTANT("IBA is doing one iteration");
        if (enableDropout) {
            mOptimizer->initializeOptimization(0);
        }
        mOptimizer->optimize(1);
    }
*/

}

void CML::Optimization::G2O::IndirectBundleAdjustment::apply() {
    signalMethodStart("G2O::IndirectBundleAdjustment::apply");

    if (mOptimizer == nullptr) {
        return;
    }

    PointHashMap<List<Pair<PFrame, Vector3>>> pointsCoordinates;
    if (mAdjustDirectPoints.b()) {
        for (auto pKF : lLocalKeyFrames) {
            for (auto pPoint : pKF->getMapPointsApparitions()) {
                if (pPoint->isGroup(getMap().DIRECTGROUP)) {
                    pointsCoordinates[pPoint] = List<Pair<PFrame, Vector3>>();
                    // pointsCoordinates[pPoint].emplace_back(pPoint->getWorldCoordinate().absolute());
                }
            }
        }
        for (auto &[pPoint, coordinates] : pointsCoordinates) {
            for (auto frame : pPoint->getDirectApparitions()) {
                coordinates.emplace_back(frame, pPoint->getWorldCoordinate().relative(frame->getCamera()));
            }
        }
    }

    //Keyframes
    for (auto pKF : lLocalKeyFrames) {
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(mOptimizer->vertex(pKF->getId() + idOffset));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->setCamera(Camera(SE3quat.translation().cast<scalar_t>(), SE3quat.rotation().matrix().cast<scalar_t>()));
    }


    //Points
    for (auto pMP : lLocalIndirectPoints)
    {
        size_t vertexId = pMP->getId()+maxKFid + 1;
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(mOptimizer->vertex(vertexId));
        if (vPoint == nullptr) {
            continue;
        }
        pMP->setWorldCoordinate(WorldPoint::fromAbsolute(vPoint->estimate().cast<scalar_t>()));
        // pMP->updateNormalAndDepth();
    }

    // Edges
    for(size_t i=0, iend=vpEdges.size(); i < iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
        auto [frame, point] = vpEdgesPairs[i];

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            if(mRemoveEdge.b() && point->getReferenceFrame() != frame) {
                frame->removeMapPoint(point);
            }
        }

    }

    if (mAdjustDirectPoints.b()) {

        for (auto [pPoint, coordinates] : pointsCoordinates) {
            if (coordinates.size() == 0) {
                continue;
            }
            Vector3 mean = Vector3::Zero();
            for (auto [frame, coordinate] : coordinates) {
                mean += WorldPoint::fromRelativeCoordinates(coordinate, frame->getCamera()).absolute();
            }
            mean /= coordinates.size();
            pPoint->setWorldCoordinate(WorldPoint::fromAbsolute(mean));
        }

    }

}
