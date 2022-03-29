#include "cml/optimization/ceres/CeresFunctions.h"
#include "cml/evaluation/Alignment.h"
#include "cml/map/Camera.h"

namespace CML::Evaluation {
    bool isSaved = false;
    double savedTranslation[3];
    double savedRotation[4];
}

double CML::Evaluation::align(const List<Camera> &input, const List<Optional<Camera>> &groundtruth, List<Camera> &output) {

    return 0;

    if (input.size() == 0) {
        return 0;
    }

    assertThrow(input.size() == groundtruth.size() && input.size() == output.size(), "The number of input must be the same as the number of groundtruth and number of output");

    ceres::Problem::Options problemOptions;
    problemOptions.enable_fast_removal = true;
    ceres::Problem problem(problemOptions);

    double *translation = new double[3];
    problem.AddParameterBlock(translation, 3);
    //problem.SetParameterBlockConstant(translation);

    double *quaternion = new double[4];
    problem.AddParameterBlock(quaternion, 4);
    ceres::QuaternionParameterization *parameterization = new ceres::QuaternionParameterization(); // w, x, y, z
    problem.SetParameterization(quaternion, parameterization);

    if (!isSaved) {
        translation[0] = -input[0].eye()[0];
        translation[1] = -input[0].eye()[1];
        translation[2] = -input[0].eye()[2];
        quaternion[0] = 1;
        quaternion[1] = 0;
        quaternion[2] = 0;
        quaternion[3] = 0;
    } else {
        translation[0] = savedTranslation[0];
        translation[1] = savedTranslation[1];
        translation[2] = savedTranslation[2];
        quaternion[0] = savedRotation[0];
        quaternion[1] = savedRotation[1];
        quaternion[2] = savedRotation[2];
        quaternion[3] = savedRotation[3];
    }

//    problem.SetParameterBlockConstant(quaternion);

    double *scaling = new double[1];
    scaling[0] = 1;
    problem.AddParameterBlock(scaling, 1);

    int numCorrespondances = 0;

    for (size_t i = 0; i < input.size(); i++) {

        if (!groundtruth[i].has_value()) {
            continue;
        }

        double *camera_center = new double[3];
        camera_center[0] = input[i].eye()[0];
        camera_center[1] = input[i].eye()[1];
        camera_center[2] = input[i].eye()[2];
        problem.AddParameterBlock(camera_center, 3);
        problem.SetParameterBlockConstant(camera_center);

        double *gt_center = new double[3];
        gt_center[0] = groundtruth[i].value().eye()[0];
        gt_center[1] = groundtruth[i].value().eye()[1];
        gt_center[2] = groundtruth[i].value().eye()[2];
        problem.AddParameterBlock(gt_center, 3);
        problem.SetParameterBlockConstant(gt_center);

        double *camera_front = new double[3];
        camera_front[0] = input[i].look()[0];
        camera_front[1] = input[i].look()[1];
        camera_front[2] = input[i].look()[2];
        problem.AddParameterBlock(camera_front, 3);
        problem.SetParameterBlockConstant(camera_front);

        double *gt_front = new double[3];
        gt_front[0] = groundtruth[i].value().look()[0];
        gt_front[1] = groundtruth[i].value().look()[1];
        gt_front[2] = groundtruth[i].value().look()[2];
        problem.AddParameterBlock(gt_front, 3);
        problem.SetParameterBlockConstant(gt_front);

        auto transformErrorA = Optimization::Ceres::AlignmentError::create();
        problem.AddResidualBlock(transformErrorA, nullptr, camera_center, translation, quaternion, scaling, gt_center);

        auto transformErrorB = Optimization::Ceres::AlignmentError::create();
        problem.AddResidualBlock(transformErrorB, nullptr, camera_front, translation, quaternion, scaling, gt_front);

        numCorrespondances++;

    }

    ceres::Solver::Options options;
    options.logging_type = ceres::SILENT;
    options.minimizer_progress_to_stdout = false;
    //options.minimizer_type = ceres::LINE_SEARCH;
    options.max_num_iterations = 10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        logger.error("Ceres Message : (evaluation)\n" + summary.message);
        //return std::numeric_limits<float>::infinity();
    } else {
        //if (!summary.message.empty()) logger.info("Ceres Message : (evaluation)\n" + summary.message);
    }

    for (size_t i = 0; i < input.size(); i++) {

        double *cc = new double[3];
        cc[0] = input[i].eye()[0];
        cc[1] = input[i].eye()[1];
        cc[2] = input[i].eye()[2];
        problem.AddParameterBlock(cc, 3);
        problem.SetParameterBlockConstant(cc);

        double *cf = new double[3];
        cf[0] = input[i].look()[0];
        cf[1] = input[i].look()[1];
        cf[2] = input[i].look()[2];
        problem.AddParameterBlock(cf, 3);
        problem.SetParameterBlockConstant(cf);

        Optimization::Ceres::AlignmentError::evaluate(cc, translation, quaternion, scaling, cc);
        Optimization::Ceres::AlignmentError::evaluate(cf, translation, quaternion, scaling, cf);

        output[i] = Camera::fromDirectionVector(Vector3(cc[0], cc[1], cc[2]), Vector3 (cf[0], cf[1], cf[2]));

        delete[] cc;
        delete[] cf;
    }


    savedTranslation[0] = translation[0];
    savedTranslation[1] = translation[1];
    savedTranslation[2] = translation[2];
    savedRotation[0] = quaternion[0];
    savedRotation[1] = quaternion[1];
    savedRotation[2] = quaternion[2];
    savedRotation[3] = quaternion[3];

    return summary.final_cost;

}