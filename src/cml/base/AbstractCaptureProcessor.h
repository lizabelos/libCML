#ifndef CML_ABSTRACTCAPTUREPROCESSOR
#define CML_ABSTRACTCAPTUREPROCESSOR

#include "cml/capture/AbstractCapture.h"
#include "AbstractFunction.h"

namespace CML {

    class AbstractCaptureProcessor {

    public:
        virtual Ptr<CaptureImage, Nullable> getLastCaptureFrame() = 0;

    };

}

#endif