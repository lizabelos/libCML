#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>
#include <cml/robust/RobustFundamental8Points.h>
#include <cml/robust/RobustHomography.h>
#include <cml/robust/CameraChecker.h>

namespace CML::Robust {

    class RobustRaulmurInitializer : public AbstractFunction {

    public:
        RobustRaulmurInitializer(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent), mRobustFundamental(this), mRobustHomography(this), mTriangulator(this), mCameraChecker(parent) {

        }

        bool track(Map &map, PFrame frameA, PFrame frameB, List<Matching> matching);

        inline std::string getName() final {
            return "Robust Raulmur";
        }

        void viewOnReconstruction(DrawBoard &drawBoard) final {
            List<Camera> cameras;
            {
                LockGuard lg(mLastBestHypothesisMutex);
                cameras = mLastBestHypothesis;
            }

            drawBoard.lineWidth(2);
            drawBoard.color(1,0,0);
            for (auto camera : cameras) {
                drawBoard.paintCamera(camera);
            }

        }

    private:
        RobustFundamental8Points mRobustFundamental;
        RobustHomography mRobustHomography;
        Hartley2003Triangulation mTriangulator;
        CameraChecker mCameraChecker;

        PStatistic mRatioHomography = createStatistic("Ratio Homography");
        PStatistic mInliers = createStatistic("Inliers");

        Mutex mLastBestHypothesisMutex;
        List<Camera> mLastBestHypothesis;

    };

}