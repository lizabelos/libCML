//
// Created by belosth on 10/12/2019.
//

#ifndef CML_EVALUATION_H
#define CML_EVALUATION_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "cml/config.h"

namespace CML {

    class EvaluationFrame {

    public:
        virtual scalar_t getEvaluationTimestamp() = 0;

        virtual Camera getEvaluationPosition() = 0;

    };

    typedef struct {
        size_t indexA, indexB;
        scalar_t distance;
    } EvaluationFrameMatch;

    typedef struct {
        scalar_t distance;
        scalar_t angle;
        int stampData0, stampData1;
        int stampModel0, stampModel1;
    } rpe_t;

    void absoluteTrajectoryError(const std::vector<EvaluationFrame*> &model, const std::vector<EvaluationFrame*> &data, Vector3 &trans, Matrix33 &rot, std::vector<Vector3> &aligned, std::vector<scalar_t> &transError);
    void absoluteTrajectoryError(std::vector<Vector3> model, std::vector<Vector3> data, Vector3 &trans, Matrix33 &rot, std::vector<Vector3> &aligned, std::vector<scalar_t> &transError);

    std::vector<rpe_t> relativePoseError(const std::vector<EvaluationFrame*> &model, const std::vector<EvaluationFrame*> &data);

}

#endif //CML_EVALUATION_H
