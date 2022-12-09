#include "cml/optimization/g2o/IndirectCameraOptimizer.h"
#include "cml/optimization/g2o/g2oconfig.h"

CML::Optimization::G2O::IndirectCameraOptimizerResult CML::Optimization::G2O::IndirectCameraOptimizer::optimize(PFrame frame, const Optional<Camera> &camera, const List<Matching> &matchings, List<bool> &outliers, bool computeCovariance) {

    IndirectCameraOptimizerResult result;

    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
            new DefaultG2OSolverWithCovariance<g2o::BlockSolver_6_3::PoseMatrixType>()
            );

    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(
            new g2o::BlockSolver_6_3(std::move(linearSolver))
    );

    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr)));

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(g2o::SE3Quat(frame->getCamera().getRotationMatrix().cast<number_t>(), frame->getCamera().getTranslation().cast<number_t>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    vSE3->setMarginalized(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = matchings.size();

    List<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    List<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);

    Matrix33 K = frame->getK(0);
    scalar_t fx = K(0,0);
    scalar_t fy = K(1,1);
    scalar_t cx = K(0, 2);
    scalar_t cy = K(1, 2);

    outliers = List<bool>();
    outliers.resize(N, false);

    for(int i=0; i<N; i++)
    {
        OptPPoint pMP = matchings[i].getMapPoint();
        if (pMP.isNull()) {
            continue;
        }

        // Monocular observation
        nInitialCorrespondences++;
        outliers[i] = false;

        DistortedVector2d obs = matchings[i].getFeaturePoint(frame).point0();

        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->getWorldCoordinate().absolute().cast<number_t>());
        vPoint->setId(i + 1);
        vPoint->setMarginalized(true);
        vPoint->setFixed(true);
        bool r = optimizer.addVertex(vPoint);
        assertThrow(r, "Vertex already exist");

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i + 1)));
        e->setMeasurement(obs.cast<number_t>());
        //const float invSigma2 = pMP->getReferenceCorner().level() * pMP->getUncertainty();
        scalar_t scaleFactor = matchings[i].getFeaturePoint(frame).processScaleFactorFromLevel();
        scalar_t invSigma2 = 1.0 / (scaleFactor * scaleFactor);
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
        e->setLevel(0);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        e->fx = fx;
        e->fy = fy;
        e->cx = cx;
        e->cy = cy;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vnIndexEdgeMono.push_back(i);


    }

    assertDeterministic("Number of G2O tracker initial correspondance", nInitialCorrespondences);

    if(nInitialCorrespondences < 3) {
        return result;
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const int its[4]={10,10,10,10};

    Camera initialCamera = frame->getCamera();
    if (camera.has_value()) {
        initialCamera = camera.value();
    }

    scalar_t chi2Mono = 5.991;
    int nBad = 0;

    for(size_t it = 0; it < 4; it++)
    {

        vSE3->setEstimate(g2o::SE3Quat(initialCamera.getRotationMatrix().cast<number_t>(), initialCamera.getTranslation().cast<number_t>()));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = evaluteOutliers(vpEdgesMono, vnIndexEdgeMono, outliers, chi2Mono);

        if((nInitialCorrespondences-nBad)<5) {
            return result;
        }

        if (it == 2) {
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
                e->setRobustKernel(nullptr);
            }
        }

        if(optimizer.edges().size() < 10) {
            return result;
        }




    }


    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();

    result.camera = Camera(SE3quat_recov.translation().cast<scalar_t>(),SE3quat_recov.rotation().matrix().cast<scalar_t>());

    if (computeCovariance) {
        // Compute covariance
        g2o::SparseBlockMatrixX spinv;
        bool computeMarginalsSucceed = optimizer.computeMarginals(spinv, optimizer.vertex(0));
        if (!computeMarginalsSucceed) {
            CML_LOG_ERROR("Can't compute marginals");
            return result;
        }
        auto block = spinv.block(0, 0);
        if (block == nullptr) {
            CML_LOG_ERROR("Can't compute marginals");
            return result;
        }

        result.covariance = spinv.block(0, 0)->eval().diagonal().cast<scalar_t>();
    }

    result.isOk = true;

    return result;

}

CML::Optimization::G2O::IndirectCameraOptimizerResult CML::Optimization::G2O::IndirectCameraOptimizer::optimize(PFrame frame, List<PPoint> &outliersPoints, bool computeCovariance) {

    IndirectCameraOptimizerResult result;

    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver(
            new DefaultG2OSolverWithCovariance<g2o::BlockSolver_6_3::PoseMatrixType>()
    );

    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(
            new g2o::BlockSolver_6_3(std::move(linearSolver))
    );

    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr)));

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(g2o::SE3Quat(frame->getCamera().getRotationMatrix().cast<number_t>(), frame->getCamera().getTranslation().cast<number_t>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    vSE3->setMarginalized(false);
    optimizer.addVertex(vSE3);



    // Set MapPoint vertices
    List<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    List<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(100);
    vnIndexEdgeMono.reserve(100);

    const float deltaMono = sqrt(5.991);

    Matrix33 K = frame->getK(0);
    scalar_t fx = K(0,0);
    scalar_t fy = K(1,1);
    scalar_t cx = K(0, 2);
    scalar_t cy = K(1, 2);

    List<bool> outliers;
    List<PPoint> optimizedMapPoints;
    outliers.reserve(100);
    int i = 0;

    for (auto pMP : frame->getGroupMapPoints(getMap().INDIRECTGROUP))
    {

        auto index = frame->getIndex(pMP);

        if (!index.hasValidValue()) {
            continue;
        }

        // Monocular observation
        nInitialCorrespondences++;

        Corner featurePoint = frame->getFeaturePoint(index);

        DistortedVector2d obs = featurePoint.point0();

        g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
        vPoint->setEstimate(pMP->getWorldCoordinate().absolute().cast<number_t>());
        vPoint->setId(i + 1);
        vPoint->setMarginalized(true);
        vPoint->setFixed(true);
        bool r = optimizer.addVertex(vPoint);
        assertThrow(r, "Vertex already exist");

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i + 1)));
        e->setMeasurement(obs.cast<number_t>());
        scalar_t scaleFactor = featurePoint.processScaleFactorFromLevel();
        scalar_t invSigma2 = 1.0 / (scaleFactor * scaleFactor);
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
        e->setLevel(0);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        e->fx = fx;
        e->fy = fy;
        e->cx = cx;
        e->cy = cy;

        optimizer.addEdge(e);

        outliers.emplace_back(false);
        vpEdgesMono.push_back(e);
        vnIndexEdgeMono.push_back(i);
        optimizedMapPoints.emplace_back(pMP);

        i++;


    }

    assertDeterministic("Number of G2O tracker initial correspondance", nInitialCorrespondences);

    if(nInitialCorrespondences < 3) {
        return result;
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const int its[4]={10,10,10,10};

    Camera initialCamera = frame->getCamera();

    scalar_t chi2Mono = 5.991;
    int nBad = 0;

    for(size_t it = 0; it < 4; it++)
    {

        vSE3->setEstimate(g2o::SE3Quat(initialCamera.getRotationMatrix().cast<number_t>(), initialCamera.getTranslation().cast<number_t>()));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = evaluteOutliers(vpEdgesMono, vnIndexEdgeMono, outliers, chi2Mono);

        if((nInitialCorrespondences-nBad)<5) {
            return result;
        }

        if (it == 2) {
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
                e->setRobustKernel(nullptr);
            }
        }

        if(optimizer.edges().size() < 10) {
            return result;
        }

    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();

    for (size_t i = 0; i < optimizedMapPoints.size(); i++) {
        if (outliers[i]) {
            outliersPoints.emplace_back(optimizedMapPoints[i]);
        }
    }


    result.camera = Camera(SE3quat_recov.translation().cast<scalar_t>(),SE3quat_recov.rotation().matrix().cast<scalar_t>());

    // Compute covariance
    if (computeCovariance) {
        g2o::SparseBlockMatrixX spinv;
        bool computeMarginalsSucceed = optimizer.computeMarginals(spinv, optimizer.vertex(0));
        if (!computeMarginalsSucceed) {
            return result;
        }
        auto block = spinv.block(0, 0);
        if (block == nullptr) {
            return result;
        }

        result.covariance = spinv.block(0, 0)->eval().diagonal().cast<scalar_t>();
    }

    result.isOk = true;

    return result;


}

int CML::Optimization::G2O::IndirectCameraOptimizer::evaluteOutliers(List<g2o::EdgeSE3ProjectXYZ*> &vpEdges, List<size_t> vnIndexEdge, List<bool> &outliers, scalar_t chi2Threshold) {

    if (!mCheckOutliers.b()) {
        for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];
            outliers[i] = false;
            e->setLevel(0);
        }
        return 0;
    }

    int nBad = 0;
    for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];

        const size_t idx = vnIndexEdge[i];

        //if(outliers[idx])
        //{
            e->computeError();
        //}

        const float chi2 = e->chi2();

        if(!std::isfinite(chi2) || chi2 > chi2Threshold)
        {
            outliers[idx]=true;
            e->setLevel(1);
            nBad++;
        }
        else
        {
            outliers[idx]=false;
            e->setLevel(0);
        }

    }

    return nBad;

}
