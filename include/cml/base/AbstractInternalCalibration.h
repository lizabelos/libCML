#ifndef CML_ABSTRACTPHOTOMETRICCALIBRATION_H
#define CML_ABSTRACTPHOTOMETRICCALIBRATION_H

#include "AbstractCaptureProcessor.h"
#include "cml/capture/AbstractCapture.h"
#include "AbstractFunction.h"

namespace CML {

    class AbstractInternalCalibration : public AbstractCaptureProcessor {

    public:
        virtual void start(CML::AbstractCapture *capture) = 0;

        virtual void interrupt() = 0;

    };

}

#endif