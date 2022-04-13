#ifndef CML_INDIRECTBUNDLEADJUSTMENT_H
#define CML_INDIRECTBUNDLEADJUSTMENT_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/sba/edge_project_xyz.h>

namespace CML::Optimization::G2O {

    typedef struct IndirectBundleAdjustmentDeformation {
        List<scalar_t> from, to;
    } IndirectBundleAdjustmentDeformation;

    class IndirectBundleAdjustment : public AbstractFunction {

    public:
        IndirectBundleAdjustment(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

        ~IndirectBundleAdjustment() {
            if (mOptimizer != nullptr) {
                delete mOptimizer;
            }
        }

        bool localOptimize(PFrame frame, int frameGroup, bool *pbStopFlag, bool fixFrames = false);

        void apply();

        std::string getName() final {
            return "Hybrid Bundle Adjustment";
        }

        bool contain(PFrame frame) {
            return lLocalKeyFrames.count(frame) > 0;
        }

        OrderedSet<PFrame, Comparator> getAllFrames() {
            return lLocalKeyFrames;
        }

        void setNumIteration(int v) {
            mNumIteration.set(v);
        }

        void setRemoveEdge(bool v) {
            mRemoveEdge.set(v);
        }

    protected:
        void startOptimization(int num, bool enableDropout, bool onlyRobust);

    private:
        OrderedSet<PFrame, Comparator>  lLocalKeyFrames, lFixedCameras;
        Set<PPoint> lLocalIndirectPoints;
        List<g2o::EdgeSE3ProjectXYZ*> vpEdges;
        List<Pair<PFrame, PPoint>> vpEdgesPairs;
        unsigned long maxKFid = 0;
        int idOffset = 0;

        g2o::SparseOptimizer *mOptimizer = nullptr;

        Parameter mNumIteration = createParameter("numIteration", 5);
        Parameter mRefineIteration = createParameter("refineIteration", 0);
        Parameter mRemoveEdge = createParameter("removeEdge", true);
        Parameter mDropout = createParameter("dropout", 0.0);

        Parameter mTimeLimit = createParameter("timeLimit", 0.015);
        Parameter mAdjustDirectPoints = createParameter("adjustDirectPoints", false);

        Timer mTimer;
    };

}

#endif