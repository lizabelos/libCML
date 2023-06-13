#ifndef CML_MIDAS_H
#define CML_MIDAS_H

#include <cml/config.h>
#include <cml/capture/CaptureImage.h>
#include <memory>
#include <torch/script.h>

namespace CML::NN {

    class Midas {

    public:
        Midas();

        Array2D<float> operator() (const CaptureImage &input);

    private:
        torch::jit::Module module;

    };


}

#endif