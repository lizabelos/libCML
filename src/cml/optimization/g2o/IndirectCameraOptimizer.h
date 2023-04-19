#ifndef CML_INDIRECTCAMERAOPTIMIZER_H
#define CML_INDIRECTCAMERAOPTIMIZER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

#include <g2o/types/sba/edge_project_xyz.h>


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

            IndirectCameraOptimizerResult optimize(PFrame frame, const Optional<Camera> &camera, const List<Matching> &matchings, List<bool> &outliers, bool computeCovariance);

            IndirectCameraOptimizerResult optimize(PFrame frame, const List<Matching> &matchings, List<bool> &outliers, bool computeCovariance) {
                return optimize(frame, Optional<Camera>(), matchings, outliers, computeCovariance);
            }

            IndirectCameraOptimizerResult optimize(PFrame frame, List<PPoint> &outliers, bool computeCovariance);

            std::string getName() final {
                return "Indirect Camera Optimizer";
            }

            void setCheckOutliers(bool checkOutliers) {
                mCheckOutliers.set(checkOutliers);
            }

        protected:
            int evaluteOutliers(List<g2o::EdgeSE3ProjectXYZ*> &vpEdges, List<size_t> vnIndexEdge, List<bool> &outliers, scalar_t chi2Threshold);

        private:
            Parameter mCheckOutliers = createParameter("checkOutliers", true);

        };

    }

}

#endif