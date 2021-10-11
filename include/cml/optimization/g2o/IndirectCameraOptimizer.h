#ifndef CML_INDIRECTCAMERAOPTIMIZER_H
#define CML_INDIRECTCAMERAOPTIMIZER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace CML {

    namespace Optimization::G2O {

        typedef struct IndirectCameraOptimizerResult {

            IndirectCameraOptimizerResult() {
                covariance.fill(999999);
            }

            bool isOk = false;
            Camera camera;
            Vector6 covariance;

        } IndirectCameraOptimizerResult;

        class IndirectCameraOptimizer : public AbstractFunction {

        public:
            IndirectCameraOptimizer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

            IndirectCameraOptimizerResult optimize(PFrame frame, const Optional<Camera> &camera, const List<Matching> &matchings, List<bool> &outliers);

            IndirectCameraOptimizerResult optimize(PFrame frame, const List<Matching> &matchings, List<bool> &outliers) {
                return optimize(frame, Optional<Camera>(), matchings, outliers);
            }

            IndirectCameraOptimizerResult optimize(PFrame frame, List<PPoint> &outliers);

            std::string getName() final {
                return "Indirect Camera Optimizer";
            }

        protected:
            int evaluteOutliers(List<g2o::EdgeSE3ProjectXYZOnlyPose*> &vpEdges, List<size_t> vnIndexEdge, List<bool> &outliers, scalar_t chi2Threshold);

        };

    }

}

#endif