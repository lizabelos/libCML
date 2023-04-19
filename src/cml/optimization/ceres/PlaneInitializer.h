#ifndef CML_OPTIMIZATION_CERES_PLANEINITIALIZER
#define CML_OPTIMIZATION_CERES_PLANEINITIALIZER

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

namespace CML::Optimization::Ceres {

    class PlaneInitializer : public AbstractFunction {

    public:
        PlaneInitializer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {

        }

        bool initialize();

    private:


    };

}

#endif