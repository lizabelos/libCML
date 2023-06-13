#include "cml/nn/MonoDepth.h"

CML::NN::MonoDepth::MonoDepth() {
    module = torch::jit::load("../resources/monodepth.pt", at::kCPU);
}

CML::Array2D<float> CML::NN::MonoDepth::operator() (const CaptureImage &input) {

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
    Array2D<ColorRGB> image = input.getColorImage(0).children().cast<ColorRGB>();
#else
    Array2D<ColorRGB> image = input.getColorImage(0).cast<ColorRGB>();
#endif

    at::Tensor tensor_image = torch::from_blob((void*)image.data(), {1, 3, image.getWidth(), image.getHeight()}, at::kByte);
    tensor_image = tensor_image.to(at::kFloat);
    tensor_image = tensor_image / 255.0f;
    tensor_image.permute({0, 3, 2, 1});
    tensor_image.resize_({1, 3, 256, 512});

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(tensor_image);

    auto result = module.forward(inputs);
    at::Tensor tensor = result.toTuple()->elements()[0].toTensor()[0][0];

    tensor.permute({1, 0});

    tensor = tensor - tensor.min();
    tensor = tensor / tensor.max();

    CML::Array2D<float> arrayResult(512, 256);
    memcpy(arrayResult.data(), tensor.data_ptr(), sizeof(float) * 256 * 512);
    return arrayResult;

}


CML::NN::MonoDepth2::MonoDepth2() {
    module = torch::jit::load("../resources/monodepth2.pt", at::kCPU);
}

CML::Array2D<float> CML::NN::MonoDepth2::operator() (const CaptureImage &input) {

#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
    Array2D<ColorRGB> image = input.getColorImage(0).children().cast<ColorRGB>();
#else
    Array2D<ColorRGB> image = input.getColorImage(0).cast<ColorRGB>();
#endif

    at::Tensor tensor_image = torch::from_blob((void*)image.data(), {1, 3, image.getWidth(), image.getHeight()}, at::kByte);
    tensor_image = tensor_image.to(at::kFloat);
    tensor_image = tensor_image / 255.0f;
    tensor_image.permute({0, 3, 2, 1});
    tensor_image.resize_({1, 3, 192, 640});

    std::vector<torch::jit::IValue> inputs;
    inputs.emplace_back(tensor_image);

    auto result = module.forward(inputs);
    at::Tensor tensor = result.toTensor()[0][0];

    tensor.permute({1, 0});

    tensor = tensor - tensor.min();
    tensor = tensor / tensor.max();

    CML::Array2D<float> arrayResult(640, 192);
    memcpy(arrayResult.data(), tensor.data_ptr(), sizeof(float) * 640 * 192);

    arrayResult = arrayResult.resize(input.getWidth(0), input.getHeight(0));
    (arrayResult * 255).cast<unsigned char>().saveBmp("nn.bmp");

    return arrayResult;

}
