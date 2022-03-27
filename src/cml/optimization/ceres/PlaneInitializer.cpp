#include "cml/optimization/ceres/PlaneInitializer.h"
#include "cml/optimization/ceres/CeresFunctions.h"

bool CML::Optimization::Ceres::PlaneInitializer::initialize() {

    ceres::LossFunction *loss = new ceres::HuberLoss(1);

    ceres::Problem::Options problemOptions;
    problemOptions.enable_fast_removal = true;
    ceres::Problem problem(problemOptions);

    ceres::Solver::Options options;


    OrderedSet<PFrame, Comparator> frames = getMap().getFrames();

    HashMap<PPoint, double*> pointsData;
    HashMap<PFrame, Pair<double*, double*>> framesData;

    int i = 0;

    OptPFrame firstFrame;

    for (auto frame : frames) {

        for (auto [index, point] : frame->getMapPoints()) {

            if (!point->isGroup(getMap().MAPPED)) {
                continue;
            }

            point->setReferenceInverseDepth(1);

            if (framesData.count(frame) == 0) {
                frame->setCamera(Camera(Vector3(0, 0, -0.01 * i), Matrix33::Identity()));

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

                if (i == 0) {
                    problem.SetParameterBlockConstant(translation);
                    problem.SetParameterBlockConstant(quaternion);
                    firstFrame = frame;
                }

                if (i == 1) {
                    auto fixedDistanceError = FixedDistance::create(0.01);
                    problem.AddResidualBlock(fixedDistanceError, nullptr, framesData[firstFrame].first, framesData[frame].first);

                }

                i++;
            }

            if (pointsData.count(point) == 0) {
                double *position = new double[3];
                position[0] = point->getWorldCoordinate().absolute()[0];
                position[1] = point->getWorldCoordinate().absolute()[1];
                position[2] = point->getWorldCoordinate().absolute()[2];
                problem.AddParameterBlock(position, 3);
                pointsData[point] = position;
            }

            auto undistorted = frame->undistort(frame->getFeaturePoint(index).point0(), 0);
            double *groundtruth = new double[2];
            groundtruth[0] = undistorted[0];
            groundtruth[1] = undistorted[1];
            problem.AddParameterBlock(groundtruth, 2);
            problem.SetParameterBlockConstant(groundtruth);

            auto reprojectionError = CeresReprojectionErrorXYZ::create();
            problem.AddResidualBlock(reprojectionError, loss, framesData[frame].first, framesData[frame].second, pointsData[point], groundtruth);

        }

    }

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 10;


    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        logger.error("Ceres Message : (PI)\n" + summary.message);
        return false;
    } else {
        if (!summary.message.empty()) logger.info("Ceres Message : (PI)\n" + summary.message);
    }

    for (auto [frame, data] : framesData) {
        frame->setCamera(Camera(Vector3(data.first[0], data.first[1], data.first[2]), Quaternion(data.second[0], data.second[1], data.second[2], data.second[3])));
    }

    for (auto [point, data] : pointsData) {
        WorldPoint worldPoint = WorldPoint::fromAbsolute(Vector3(data[0], data[1], data[2]));
        if (point->getReferenceFrame()->getCamera().inFront(worldPoint.absolute())) {
            point->setWorldCoordinate(worldPoint);
        } else {
            // todo : add to to outliers
        }
    }

    return true;

}