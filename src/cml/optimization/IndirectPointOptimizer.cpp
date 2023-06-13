#include "cml/optimization/IndirectPointOptimizer.h"

void CML::Optimization::IndirectPointOptimizer::optimize(PPoint point) {

    WorldPoint currentCoordinate = point->getWorldCoordinate(), newCoordinate;
    scalar_t currentResidual, newResidual;
    Matrix33 currentH, newH;
    Vector3 currentB, newB;

    computeResidualAndJacobian(point, currentCoordinate, currentResidual, currentH, currentB);

    scalar_t lambda = 0.01;

    for (int i = 0; i < 10; i++) {

        Matrix33 dampedHessian = currentH;
        for (int j = 0; j < 3; j++) dampedHessian(j, j) *= (1 + lambda);
        Vector3 increment = dampedHessian.ldlt().solve(-currentB);

        newCoordinate = WorldPoint::fromAbsolute(currentCoordinate.absolute() + increment);
        computeResidualAndJacobian(point, newCoordinate, newResidual, newH, newB);

        if (newResidual < currentResidual) {
            currentResidual = newResidual;
            currentH = newH;
            currentB = newB;
            currentCoordinate = newCoordinate;
        }

    }

    point->setWorldCoordinate(newCoordinate);


}

void CML::Optimization::IndirectPointOptimizer::computeResidualAndJacobian(PPoint point, WorldPoint coordinate, scalar_t &residual, Matrix33 &H, Vector3 &b) {

    residual = 0;

    Vector3 J = Vector3::Zero();

    for (auto frame : point->getIndirectApparitions()) {

        auto apparition = frame->getFeaturePoint(point).value();

        Vector3 transformation = frame->getCamera().transform(coordinate.absolute());
        Vector2 projection = transformation.hnormalized();
        Vector2 difference = projection - frame->undistort(apparition.point0(), 0);
        scalar_t squaredNorm = difference.squaredNorm();

        residual += squaredNorm;

        for (int i = 0; i < 3; i++) {
            J(i) -= Derivative::squaredNorm(difference, frame->getCamera().derivativeOfProjectWrtPoint(transformation, i));
        }

    }

    H = J * J.transpose();
    b = J * residual * -1;

}
