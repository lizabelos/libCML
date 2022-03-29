#include "cml/optimization/ceres/IndirectBundleAdjustment.h"
#include "cml/optimization/ceres/CeresFunctions.h"

bool CML::Optimization::Ceres::IndirectBundleAdjustment::localOptimize(int frameGroupId, int pointGroupId, bool *stop) {

    ceres::LossFunction *loss = new ceres::HuberLoss(0.05);

    ceres::Problem::Options problemOptions;
    problemOptions.enable_fast_removal = true;
    ceres::Problem problem(problemOptions);

    ceres::Solver::Options options;


    OrderedSet<PFrame, Comparator> frames = getMap().getGroupFrames(frameGroupId);
    HashMap<PPoint, int> pointsCount;
    HashMap<PFrame, int> framesCount;


    HashMap<PPoint, double*> pointsData;
    HashMap<PFrame, Pair<double*, double*>> framesData;

    OptPFrame lastFrame;
    OptPFrame lastLastFrame;

    for (auto frame : frames) {

        // If the SLAM is done correctly ( which means we have enough informations to estimate each point ), we cannot have rank deficiency
        for (auto point : frame->getGroupMapPoints(pointGroupId)) {

            if (frames.find(point->getReferenceFrame()) == frames.end()) {
                continue;
            }

            if (point->getIndirectApparitionNumber() < 2) {
                continue;
            }

            if (pointsCount.count(point) == 0) {
                pointsCount[point] = 1;
            } else {
                pointsCount[point]++;
            }

            if (framesCount.count(frame) == 0) {
                framesCount[frame] = 1;
            } else {
                framesCount[frame]++;
            }

        }

    }

    if (frames.size() < 2) {
        return false;
    }

    double *intrinsics = new double[4];
    auto K =  (*frames.begin())->getK(0);
    intrinsics[0] = K(0,0);
    intrinsics[1] = K(1,1);
    intrinsics[2] = K(0, 2);
    intrinsics[3] = K(1, 2);
    problem.AddParameterBlock(intrinsics, 4);
    problem.SetParameterBlockConstant(intrinsics);

    for (auto frame : frames) {

        if (framesCount[frame] < 10) {
            // todo : marginalize this frame ?
            continue;
        }

        // If the SLAM is done correctly ( which means we have enough informations to estimate each point ), we cannot have rank deficiency
        for (auto point : frame->getGroupMapPoints(getMap().MAPPED)) {

            if (pointsCount.count(point) == 0) {
                // todo : marginalize this point ?
                continue;
            }

            if (pointsCount[point] < 2) {
                // todo : marginalize this point ?
                continue;
            }

            auto index = frame->getIndex(point);

            if (framesData.count(frame) == 0) {
                double *translation = new double[3];
                translation[0] = frame->getCamera().getTranslation()[0];
                translation[1] = frame->getCamera().getTranslation()[1];
                translation[2] = frame->getCamera().getTranslation()[2];
                problem.AddParameterBlock(translation, 3);

                double *quaternion = new double[4];
                quaternion[0] = frame->getCamera().getQuaternion().w();
                quaternion[1] = frame->getCamera().getQuaternion().x();
                quaternion[2] = frame->getCamera().getQuaternion().y();
                quaternion[3] = frame->getCamera().getQuaternion().z();
                problem.AddParameterBlock(quaternion, 4);

                ceres::QuaternionParameterization *parameterization = new ceres::QuaternionParameterization(); // w, x, y, z
                problem.SetParameterization(quaternion, parameterization);

                framesData[frame] = Pair<double*, double*>(translation, quaternion);

                lastLastFrame = lastFrame;
                lastFrame = frame;

            }

            if (pointsData.count(point) == 0) {
                // todo : test to add only one point to see what happened with the point
                // todo : maybe to fix the frame, add to as hyperparameter ? OxO
                double *position = new double[3];
                position[0] = point->getWorldCoordinate().absolute()[0];
                position[1] = point->getWorldCoordinate().absolute()[1];
                position[2] = point->getWorldCoordinate().absolute()[2];
                problem.AddParameterBlock(position, 3);
                pointsData[point] = position;
            }

            auto corner = frame->getFeaturePoint(index);
            double *groundtruth = new double[2];
            groundtruth[0] = corner.x();
            groundtruth[1] = corner.y();
            problem.AddParameterBlock(groundtruth, 2);
            problem.SetParameterBlockConstant(groundtruth);

            auto reprojectionError = CeresReprojectionErrorXYZ::create();
            problem.AddResidualBlock(reprojectionError, loss, framesData[frame].first, framesData[frame].second, pointsData[point], groundtruth, intrinsics);

        }

    }

    if (lastFrame.isNull() || lastLastFrame.isNull()) {
        return true;
    }

    {
        // We fix 7 parameters because of gauge freedom
        problem.SetParameterBlockConstant(framesData[lastFrame].first);
        problem.SetParameterBlockConstant(framesData[lastFrame].second);
        problem.SetParameterBlockConstant(framesData[lastLastFrame].first);
    //    problem.SetParameterBlockConstant(framesData[lastLastFrame].second);

   //     auto fixedDistanceError = FixedDistance::create((lastFrame->getCamera().getTranslation() - lastLastFrame->getCamera().getTranslation()).norm());
   //     problem.AddResidualBlock(fixedDistanceError, nullptr, framesData[lastFrame].first, framesData[lastLastFrame].first);


    }

    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.visibility_clustering_type = ceres::SINGLE_LINKAGE;
    options.max_num_iterations = 100;

    if (stop != nullptr) {
        // options. // TODO
    }

    logger.info("Optimizing " + std::to_string(framesData.size()) + " frames and " + std::to_string(pointsData.size()) + " points");

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        logger.error("Ceres Message : (IBA)\n" + summary.message);
        return false;
    } else {
        if (!summary.message.empty()) logger.info("Ceres Message : (IBA)\n" + summary.message);
    }

/*
    ceres::Covariance::Options covOptions;
    ceres::Covariance *covariance = new ceres::Covariance(covOptions);

    std::vector<std::pair<const double*, const double*>> covarianceBlocks;
    for (auto [point, data] : pointsData) {
        covarianceBlocks.emplace_back(data, data);
    }


    if (!covariance->Compute(covarianceBlocks, &problem)) {
        logger.error("Rank deficient jacobian matrix ! The computation of the covariance will be very slow !");
        delete covariance;
        covOptions.algorithm_type = ceres::DENSE_SVD;
        covOptions.null_space_rank = -1;
        covariance = new ceres::Covariance(covOptions);

        if (!covariance->Compute(covarianceBlocks, &problem)) {
            delete covariance;
            logger.error("Fail to compute the covariance !");
            return false;
        }

    } else {
        logger.info("Fast covariance computation succeed !");
    }
*/
    for (auto [frame, data] : framesData) {
        frame->setCamera(Camera(Vector3(data.first[0], data.first[1], data.first[2]), Quaternion(data.second[0], data.second[1], data.second[2], data.second[3])));
    }

    for (auto [point, data] : pointsData) {
        WorldPoint worldPoint = WorldPoint::fromAbsolute(Vector3(data[0], data[1], data[2]));
        if (point->getReferenceFrame()->getCamera().inFront(worldPoint.absolute())) {
            point->setWorldCoordinate(worldPoint);
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covBlock;
           // covariance->GetCovarianceBlock(data, data, covBlock.data());
           // point->setUncertainty(covBlock.diagonal().norm());
        } else {
            getMap().removeMapPoint(point); // todo : add to outliers list instead
        }
    }

   // delete covariance;

    return true;

}

bool CML::Optimization::Ceres::IndirectBundleAdjustment::optimizeSinglePoint(CML::Ptr<CML::MapPoint, false> point, const List<PFrame> &frames, bool computeCovariance) {

    if (frames.size() < 2) {
        return false;
    }

    ceres::LossFunction *loss = new ceres::HuberLoss(0.05);

    ceres::Problem::Options problemOptions;
    problemOptions.enable_fast_removal = true;
    ceres::Problem problem(problemOptions);

    ceres::Solver::Options options;

    HashMap<PPoint, double*> pointsData;
    HashMap<PFrame, Pair<double*, double*>> framesData;

    OptPFrame lastFrame;
    OptPFrame lastLastFrame;

    double *intrinsics = new double[4];
    auto K =  (*frames.begin())->getK(0);
    intrinsics[0] = K(0,0);
    intrinsics[1] = K(1,1);
    intrinsics[2] = K(0, 2);
    intrinsics[3] = K(1, 2);
    problem.AddParameterBlock(intrinsics, 4);
    problem.SetParameterBlockConstant(intrinsics);

    for (auto frame : frames) {


            auto index = frame->getIndex(point);

            if (!index.hasValidValue()) {
                continue;
            }

            if (framesData.count(frame) == 0) {
                double *translation = new double[3];
                translation[0] = frame->getCamera().getTranslation()[0];
                translation[1] = frame->getCamera().getTranslation()[1];
                translation[2] = frame->getCamera().getTranslation()[2];
                problem.AddParameterBlock(translation, 3);

                double *quaternion = new double[4];
                quaternion[0] = frame->getCamera().getQuaternion().w();
                quaternion[1] = frame->getCamera().getQuaternion().x();
                quaternion[2] = frame->getCamera().getQuaternion().y();
                quaternion[3] = frame->getCamera().getQuaternion().z();
                problem.AddParameterBlock(quaternion, 4);

                ceres::QuaternionParameterization *parameterization = new ceres::QuaternionParameterization(); // w, x, y, z
                problem.SetParameterization(quaternion, parameterization);

                framesData[frame] = Pair<double*, double*>(translation, quaternion);

                lastLastFrame = lastFrame;
                lastFrame = frame;

                problem.SetParameterBlockConstant(translation);
                problem.SetParameterBlockConstant(quaternion);

            }

            if (pointsData.count(point) == 0) {
                double *position = new double[3];
                position[0] = point->getWorldCoordinate().absolute()[0];
                position[1] = point->getWorldCoordinate().absolute()[1];
                position[2] = point->getWorldCoordinate().absolute()[2];
                problem.AddParameterBlock(position, 3);
                pointsData[point] = position;
            }

            auto corner = frame->getFeaturePoint(index);
            double *groundtruth = new double[2];
            groundtruth[0] = corner.x();
            groundtruth[1] = corner.y();
            problem.AddParameterBlock(groundtruth, 2);
            problem.SetParameterBlockConstant(groundtruth);

            auto reprojectionError = CeresReprojectionErrorXYZ::create();
            problem.AddResidualBlock(reprojectionError, loss, framesData[frame].first, framesData[frame].second, pointsData[point], groundtruth, intrinsics);



    }

    if (lastFrame.isNull() || lastLastFrame.isNull()) {
        return true;
    }

    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;
    //options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    //options.minimizer_progress_to_stdout = true;

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.visibility_clustering_type = ceres::SINGLE_LINKAGE;
    options.max_num_iterations = 100;


    logger.info("Optimizing " + std::to_string(framesData.size()) + " frames and " + std::to_string(pointsData.size()) + " points");

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        logger.error("Ceres Message : (IBA)\n" + summary.message);
        return false;
    } else {
        if (!summary.message.empty()) logger.info("Ceres Message : (IBA)\n" + summary.message);
    }

    ceres::Covariance *covariance;
    if (computeCovariance) {

        ceres::Covariance::Options covOptions;
        covariance = new ceres::Covariance(covOptions);

        std::vector<std::pair<const double *, const double *>> covarianceBlocks;
        for (auto[point, data] : pointsData) {
            covarianceBlocks.emplace_back(data, data);
        }


        if (!covariance->Compute(covarianceBlocks, &problem)) {
            logger.error("Rank deficient jacobian matrix !");
            /*delete covariance;
            covOptions.algorithm_type = ceres::DENSE_SVD;
            covOptions.null_space_rank = -1;
            covariance = new ceres::Covariance(covOptions);

            if (!covariance->Compute(covarianceBlocks, &problem)) {
                delete covariance;
                logger.error("Fail to compute the covariance !");
                return false;
            }*/
            return false;

        } else {
            logger.info("Fast covariance computation succeed !");
        }

    }


    for (auto [point, data] : pointsData) {
        WorldPoint worldPoint = WorldPoint::fromAbsolute(Vector3(data[0], data[1], data[2]));
        if (point->getReferenceFrame()->getCamera().inFront(worldPoint.absolute())) {
            point->setWorldCoordinate(worldPoint);
            if (computeCovariance) {
                Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covBlock;
                covariance->GetCovarianceBlock(data, data, covBlock.data());
                point->setUncertainty(covBlock.diagonal().norm());
            }
        } else {
            getMap().removeMapPoint(point); // todo : add to outliers list instead
        }
    }

    if (computeCovariance) {
        delete covariance;
    }

    return true;

}
