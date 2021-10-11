#ifndef INDIRECTPOINTOPTIMIZER_H
#define INDIRECTPOINTOPTIMIZER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

namespace CML {

    namespace Optimization {

        class IndirectPointOptimizer : public AbstractFunction {

        public:
            IndirectPointOptimizer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

            void optimize(PPoint point);

        protected:
            void computeResidualAndJacobian(PPoint point, WorldPoint coordinate, scalar_t &residual, Matrix33 &H, Vector3 &b);

        };

    }

}

#endif