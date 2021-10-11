#include "cml/robust/CameraChecker.h"
#include "cml/maths/Utils.h"

CML::Robust::CameraChecker::CameraChecker(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent) {

}

int CML::Robust::CameraChecker::check(PFrame frame1,
                                           PFrame frame2,
                                           const List<Matching> &matchings,
                                           const List<Vector3> &triangulations,
                                           float th2,
                                           List<bool> &inliers,
                                           float &parallax) {

    List<UndistortedVector2d> x1;
    List<UndistortedVector2d> x2;

    x1.resize(matchings.size());
    x2.resize(matchings.size());

    for (size_t i = 0; i < matchings.size(); i++) {
        x1[i] = matchings[i].getUndistortedA(frame1, 0);
        x2[i] = matchings[i].getUndistortedB(frame2, 0);
    }

    return check(frame1->getCamera(), frame2->getCamera(), x1, x2, triangulations, th2, inliers, parallax);

}

int CML::Robust::CameraChecker::check(const Camera &camera1,
                                           const Camera &camera2,
                                           const List<UndistortedVector2d> &x1,
                                           const List<UndistortedVector2d> &x2,
                                           const List<Vector3> &triangulations,
                                           float th2,
                                           List<bool> &inliers,
                                           float &parallax) {

    assertThrow(x1.size() == x2.size(), "The number of points does not correspond");

    inliers.resize(x1.size(), false);
    List<float> allParallax;

    int nGood = 0;

    for (size_t i = 0; i < x1.size(); i++) {

        if (!triangulations[i].allFinite()) {
            continue;
        }

        if (!camera1.inFront(triangulations[i]) || !camera2.inFront(triangulations[i])) {
            continue;
        }

        Vector2 projectionA = camera1.project(triangulations[i]);
        Vector2 projectionB = camera2.project(triangulations[i]);

        scalar_t errorA = (projectionA - x1[i]).squaredNorm();
        scalar_t errorB = (projectionB - x2[i]).squaredNorm();

        if (errorA > th2 || errorB > th2) {
            continue;
        }

        float cosParallax = camera1.parallax(camera2, triangulations[i]);
        allParallax.emplace_back(cosParallax);

        nGood++;

        if (cosParallax < 0.99998) {
            inliers[i] = true;
        }

    }

    if (nGood > 0) {
        parallax = median(allParallax);
    } else {
        parallax = 0;
    }

    return nGood;

}

