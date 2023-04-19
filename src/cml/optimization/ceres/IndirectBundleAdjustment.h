#ifndef CML_OPTIMIZATION_CERES_INDIRECTBUNDLEADJUSTMENT
#define CML_OPTIMIZATION_CERES_INDIRECTBUNDLEADJUSTMENT

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

namespace CML::Optimization::Ceres {

    class IndirectBundleAdjustment : public AbstractFunction {

    public:
        IndirectBundleAdjustment(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {

        }

        bool localOptimize(int frameGroupId, int pointGroupId, bool *stop = nullptr);

        bool optimizeSinglePoint(PPoint point, const List<PFrame> &frames, bool computeCovariance = false);

    private:


    };

}

#endif