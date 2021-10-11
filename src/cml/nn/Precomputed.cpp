#include "cml/nn/Precomputed.h"
#include "cml/image/Array2D.h"
#include "cml/capture/CaptureImage.h"
#include "cml/map/InternalCalibration.h"

CML::NN::Precomputed::Precomputed(std::string suffix) : mSuffix(suffix) {

}

CML::FloatImage CML::NN::Precomputed::load(const CaptureImage &captureImage) {
    FloatImage image = loadGrayImage(captureImage.getPath() + mSuffix).cast<float>();
    image = captureImage.getInternalCalibration().removeDistortion(image);
    return image;
}