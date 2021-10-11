#ifndef CML_FAST_H
#define CML_FAST_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/capture/CaptureImage.h>

namespace CML::Features {

    typedef enum FASTType {
        FAST_9 = 9,
        FAST_10 = 10,
        FAST_11 = 11,
        FAST_12 = 12
    } FASTType;

    class FAST : public AbstractFunction {

    public:
        FAST(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent) {

        }

        static void compute(const GrayImage &frame, List<Corner> &corners, int threshold, FASTType type = FAST_9, bool nonmaxSuppression = true);

    };

}

#endif