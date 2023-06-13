//
// Created by belos on 15/08/2019.
//

#include "cml/optimization/Triangulation.h"

CML::Vector3 CML::Hartley1997LinearTriangulation::triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &xA, const UndistortedVector2d &xB) {
    Matrix44 pA = cameraA.normalizedCameraMatrix44(); // todo : nullspace camera matrix ?
    Matrix44 pB = cameraB.normalizedCameraMatrix44();

    Matrix<4, 3> A;
    A << xA.x() * pA(2, 0) - pA(0, 0) , xA.x() * pA(2, 1) - pA(0, 1) , xA.x() * pA(2, 2) - pA(0, 2) ,
            xA.y() * pA(2, 0) - pA(1, 0) , xA.y() * pA(2, 1) - pA(1, 1) , xA.y() * pA(2, 2) - pA(1, 2) ,
            xB.x() * pB(2, 0) - pB(0, 0), xB.x() * pB(2, 1) - pB(0, 1), xB.x() * pB(2, 2) - pB(0, 2),
            xB.y() * pB(2, 0) - pB(1, 0), xB.y() * pB(2, 1) - pB(1, 1), xB.y() * pB(2, 2) - pB(1, 2);

    Matrix<4, 1> B;
    B <<    -(xA.x() * pA(2, 3) - pA(0, 3)),
            -(xA.y() * pA(2, 3) - pA(1, 3)),
            -(xB.x() * pB(2, 3) - pB(0, 3)),
            -(xB.y() * pB(2, 3) - pB(1, 3));

    Matrix<3, 1> X;
    X = A.jacobiSvd().solve(B);
    return X;
}

CML::Vector3 CML::Hartley2003Triangulation::triangulateNCam(const List<Camera> &cameras, const List<UndistortedVector2d> &undistortedVectors) {

    if (cameras.size() < 2) {
        abort();
    }

    Matrix<Dynamic, Dynamic> A(3 * cameras.size(), 4);

    for (size_t k = 0; k < 4; k++) {

        for (size_t i = 0; i < cameras.size(); i++) {

            Matrix44 p = cameras[i].normalizedCameraMatrix44();
            UndistortedVector2d x = undistortedVectors[i];

            A(i * 3 + 0, k) = x(0) * p(2, k) - p(0, k);
            A(i * 3 + 1, k) = x(1) * p(2, k) - p(1, k);
            A(i * 3 + 2, k) = x(0) * p(1, k) - x(1) * p(0, k);

        }
    }

    Eigen::JacobiSVD<Matrix<Dynamic, Dynamic>> svd(A, Eigen::ComputeThinV);
    Matrix<4, 4> V = svd.matrixV();

    Vector3 r = V.block<4, 1>(0, 3).hnormalized();

    // assertThrow(pA(0, 0) > -0.5, "test");

    return r;
}

void CML::Hartley2003Triangulation::triangulateNCam(PPoint mapPoint, List<PFrame> frames) {

    if (frames.size() < 2) {
        return;
    }

    List<Camera> cameras;
    List<UndistortedVector2d> undistortedVectors;

    for (size_t i = 0; i < frames.size(); i++) {
        cameras.emplace_back(frames[i]->getCamera());
        undistortedVectors.emplace_back(frames[i]->undistort(frames[i]->getFeaturePoint(mapPoint).value().point(0), 0));
    }

    mapPoint->setWorldCoordinate(WorldPoint::fromAbsolute(triangulateNCam(cameras, undistortedVectors)));

}

CML::Vector3 CML::Hartley2003Triangulation::triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &xA, const UndistortedVector2d &xB) {


    Matrix44 pA = cameraA.normalizedCameraMatrix44();
    Matrix44 pB = cameraB.normalizedCameraMatrix44();

    Matrix<6, 4> A;

    for (size_t k = 0; k < 4; k++) {

        A(0, k) = xA(0) * pA(2, k) - pA(0, k);
        A(1, k) = xA(1) * pA(2, k) - pA(1, k);
        A(2, k) = xA(0) * pA(1, k) - xA(1) * pA(0, k);

        A(3, k) = xB(0) * pB(2, k) - pB(0, k);
        A(4, k) = xB(1) * pB(2, k) - pB(1, k);
        A(5, k) = xB(0) * pB(1, k) - xB(1) * pB(0, k);
    }

    Eigen::JacobiSVD<Matrix<Dynamic, Dynamic>> svd(A, Eigen::ComputeThinV);
    Matrix<4, 4> V = svd.matrixV();

    Vector3 r = V.block<4, 1>(0, 3).hnormalized();

    // assertThrow(pA(0, 0) > -0.5, "test");

    return r;
}