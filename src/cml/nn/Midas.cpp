#include "cml/nn/Midas.h"

CML::NN::Midas::Midas() {
    logger.info("Loading Midas model from 'resources/midas-model-small-70d6b9c8.pt'...");
    module = torch::jit::load("../resources/midas-model-small-70d6b9c8.pt", at::kCPU);
}

CML::Array2D<float> CML::NN::Midas::operator() (const CaptureImage &input) {

    //Array2D<float> image = input.getGrayImage(0).children().cast<float>();
    Array2D<float> image = Array2D<float>::from(input.getGrayImage(0));

    at::Tensor tensor_image = torch::from_blob((void*)image.data(), {1, 1, image.getWidth(), image.getHeight()}, at::kFloat);
    tensor_image = tensor_image.to(at::kFloat);
    tensor_image = tensor_image / 255.0f;
    tensor_image.permute({0, 3, 2, 1});
    tensor_image.resize_({1, 1, 256, 512});

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
