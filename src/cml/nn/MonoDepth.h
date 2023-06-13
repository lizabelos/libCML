//
// Created by tbelos on 18/04/19.
//


#ifndef CML_MONODEPTH_H
#define CML_MONODEPTH_H

#include <cml/config.h>
#include <cml/capture/CaptureImage.h>
#include <memory>
#include <torch/script.h>

namespace CML::NN {

    class MonoDepth {

    public:
        MonoDepth();

        Array2D<float> operator() (const CaptureImage &input);

    private:
        torch::jit::Module module;

    };

    class MonoDepth2 {

    public:
        MonoDepth2();

        Array2D<float> operator() (const CaptureImage &input);

    private:
        torch::jit::Module module;

    };

}


#endif //CML_MONODEPTH_H

