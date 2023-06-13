#ifndef CML_CAMERACHECKER_H
#define CML_CAMERACHECKER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/optimization/Triangulation.h>

namespace CML::Robust {

    class CameraChecker : public AbstractFunction {

    public:
        explicit CameraChecker(Ptr<AbstractFunction, Nullable> parent);

        int check(PFrame frame1,
                  PFrame frame2,
                  const List<Matching> &matchings,
                  const List<Vector3> &triangulations,
                  float th2,
                  List<bool> &inliers,
                  float &parallax);

        int check(const Camera &camera1,
                  const Camera &camera2,
                  const List<UndistortedVector2d> &x1,
                  const List<UndistortedVector2d> &x2,
                  const List<Vector3> &triangulations,
                  float th2,
                  List<bool> &inliers,
                  float &parallax);

        inline std::string getName() final {
            return "Robust Camera Checker";
        }


    private:
        PStatistic mMatchingNumber = createStatistic("Matching number");
        PStatistic mInliers = createStatistic("Inliers");
        PStatistic mAlmosts = createStatistic("Almost inliers");
        PStatistic mReprojectionOutlier = createStatistic("Reprojection outlier");
        PStatistic mReprojectionErrorMean = createStatistic("Reprojection error mean");
        PStatistic mParallaxOutlier = createStatistic("Parallax outlier");
        PStatistic mParallaxMean = createStatistic("Parallax mean");
        PStatistic mBehindOutlier = createStatistic("Behind outlier");

    };

}

#endif