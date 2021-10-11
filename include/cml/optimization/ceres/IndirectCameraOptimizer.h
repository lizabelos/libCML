#ifndef CML_OPTIMIZATION_CERES_INDIRECTCAMERAOPTIMIZER
#define CML_OPTIMIZATION_CERES_INDIRECTCAMERAOPTIMIZER

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

namespace CML::Optimization::Ceres {

    class IndirectCameraOptimizer : public AbstractFunction {

    public:
        IndirectCameraOptimizer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {

        }

        Optional<Camera> localize(PFrame frame, const List<Matching> &matchings);

    private:


    };

}

#endif