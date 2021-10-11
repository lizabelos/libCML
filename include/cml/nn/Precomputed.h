#ifndef CML_PRECOMPUTED_H
#define CML_PRECOMPUTED_H

#include <cml/config.h>

namespace CML::NN {

    class Precomputed {

    public:
        Precomputed(std::string suffix);

        FloatImage load(const CaptureImage &captureImage);

    private:
        const std::string mSuffix;

    };

}

#endif