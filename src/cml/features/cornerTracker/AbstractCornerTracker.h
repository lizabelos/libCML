#ifndef CML_ABSTRACTCORNERTRACKER_H
#define CML_ABSTRACTCORNERTRACKER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

namespace CML::Features {

    template <typename Descriptor> class AbstractCornerTracker : public AbstractFunction {

    public:
        AbstractCornerTracker(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent) {

        }

        std::string getName() override {
            return "Abstract Corner Tracker";
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) override {
            /*List<Corner> corners;
            if (frame == mLastFrameA) {
                corners = mLastCornersA;
            } else if (frame == mLastFrameB) {
                corners = mLastCornersB;
            } else {
                return;
            }*/
            List<Matching> lastMatching;

            {
                LockGuard lg(mViewMutex);
                lastMatching = mLastMatching;
            }

            drawBoard.lineWidth(1);
            drawBoard.pointSize(3);

            for (const Matching &matching : lastMatching) {
                DistortedVector2d fA = matching.noAssertGetFeaturePointA().point(0);
                DistortedVector2d fB = matching.noAssertGetFeaturePointB().point(0);
                drawBoard.color(0,1,1);
                drawBoard.segment((Eigen::Vector2f)fA.cast<float>(), (Eigen::Vector2f)fB.cast<float>());
                drawBoard.color(1, 1, 0);
                drawBoard.point((Eigen::Vector2f)fA.cast<float>());
                //drawBoard.color(1, 0, 1);
                //drawBoard.point((Eigen::Vector2f)fB.cast<float>());

            }

        }

    protected:
        void setLastResult(PFrame frameA, const List<Corner> &cornersA, PFrame frameB, const List<Corner> &cornersB, const List<Matching> &result) {
            LockGuard lg(mViewMutex);

            mLastMatching = result;
            mNumMatching->addValue(result.size());
            int mappedMatching = 0;
            for (auto matching : result) if (matching.getMapPoint().isNotNull()) mappedMatching++;
            mNumMatchingMapped->addValue(mappedMatching);
        }

        void setLastResult(const List<Matching> &result) {
            LockGuard lg(mViewMutex);
            mLastMatching = result;
        }

    private:
        Mutex mViewMutex;

        List<Matching> mLastMatching;

        PStatistic mNumMatching = createStatistic("Matching Number");
        PStatistic mNumMatchingMapped  = createStatistic("Mapped Matching number");

    };

}

#endif