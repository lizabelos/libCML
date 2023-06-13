#include "cml/optimization/ceres/IndirectCameraOptimizer.h"
#include "cml/optimization/ceres/CeresFunctions.h"

CML::Optional<CML::Camera> CML::Optimization::Ceres::IndirectCameraOptimizer::localize(PFrame frame, const List<Matching> &matchings) {

    ceres::LossFunction *loss = new ceres::HuberLoss(0.05);

    ceres::Problem::Options problemOptions;
    problemOptions.enable_fast_removal = true;
    ceres::Problem problem(problemOptions);

    ceres::Solver::Options options;


    PointHashMap<double*> pointsData;
    FrameHashMap<Pair<double*, double*>> framesData;

    {

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

    }

    for (const Matching &matching : matchings) {

        OptPPoint point = matching.getMapPoint();

        if (point.isNull()) {
            continue;
        }

        if (pointsData.count(point) == 0) {
            double *position = new double[3];
            position[0] = point->getWorldCoordinate().absolute()[0];
            position[1] = point->getWorldCoordinate().absolute()[1];
            position[2] = point->getWorldCoordinate().absolute()[2];
            problem.AddParameterBlock(position, 3);
            problem.SetParameterBlockConstant(position);
            pointsData[point] = position;
        }

        auto undistorted = frame->undistort(frame->getFeaturePoint(point).value().point0(), 0);
        double *groundtruth = new double[2];
        groundtruth[0] = undistorted[0];
        groundtruth[1] = undistorted[1];
        problem.AddParameterBlock(groundtruth, 2);
        problem.SetParameterBlockConstant(groundtruth);

        auto reprojectionError = CeresReprojectionErrorXYZ::create();
        problem.AddResidualBlock(reprojectionError, loss, framesData[frame].first, framesData[frame].second, pointsData[point], groundtruth);

    }

    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.visibility_clustering_type = ceres::SINGLE_LINKAGE;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        CML_LOG_ERROR("Ceres Message : (ICO::localize)\n" + summary.message);
        return Optional<Camera>();
    } else {
        if (!summary.message.empty()) CML_LOG_INFO("Ceres Message : (ICO::localize)\n" + summary.message);
    }

    return Camera(Vector3(framesData[frame].first[0], framesData[frame].first[1], framesData[frame].first[2]), Quaternion(framesData[frame].second[0], framesData[frame].second[1], framesData[frame].second[2], framesData[frame].second[3]));

}