#ifndef CML_INDIRECTBUNDLEADJUSTMENT_H
#define CML_INDIRECTBUNDLEADJUSTMENT_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

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

        bool localOptimize(PFrame frame, int frameGroup, bool *pbStopFlag);

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

    private:
        OrderedSet<PFrame, Comparator>  lLocalKeyFrames, lFixedCameras;
        Set<PPoint, Hasher> lLocalIndirectPoints;
        List<g2o::EdgeSE3ProjectXYZ*> vpEdges;
        List<Pair<PFrame, PPoint>> vpEdgesPairs;
        unsigned long maxKFid = 0;
        int idOffset = 0;

        g2o::SparseOptimizer *mOptimizer = nullptr;

        Parameter mNumIteration = createParameter("numIteration", 5);
        Parameter mRefineIteration = createParameter("refineIteration", 0);
        Parameter mRemoveEdge = createParameter("removeEdge", true);

    };

}

#endif