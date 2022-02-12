#include "cml/capture/TartanairCapture.h"
#include "cml/map/InternalCalibration.h"

CML::TartanairCapture::TartanairCapture(const std::string &path) {
    mImages = listDirectory(path, "png");
    if (mImages.size() == 0) {
        throw std::runtime_error("No images");
    }
    mTimes.resize(mImages.size());
    std::sort(mImages.begin(), mImages.end());
    for (size_t i = 0; i < mImages.size(); i++) {
        mTimes[i] = (float)i / 30.0;
    }

    std::string leftImagePath = mImages[mCurrentImage];
    Image image = loadPngImage(leftImagePath).second;

    mVignette = Array2D<float>(image.getWidth(), image.getHeight(), 1);
    float w = image.getWidth();
    float h = image.getHeight();
    mVignetteMax = 1;
    mCalibration = new InternalCalibration(PinholeUndistorter().scaleAndRecenter(Vector2(w, h), Vector2(-0.5, -0.5)), Vector2(w, h));
    mCaptureImageGenerator = new CaptureImageGenerator(image.getWidth(), image.getHeight());

    mIsInit = true;
}

CML::TartanairCapture::~TartanairCapture() {

}

bool CML::TartanairCapture::isInit() {
    return mIsInit;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::TartanairCapture::multithreadNext() {
    if (mCurrentImage == mImages.size()) {
        return Ptr<CaptureImage, Nullable>();
    }

    CaptureImageMaker maker = mCaptureImageGenerator->create();

    maker.setImage(loadPngImage(mImages[mCurrentImage]).second);

    maker.setPath(mImages[mCurrentImage])
            .setTime(mTimes[mCurrentImage])
            .setCalibration(mCalibration)
            .setLut(&mLut)
            .setInverseVignette(mVignette);

    mCurrentImage++;

    return maker.generate();
}

int CML::TartanairCapture::remaining() {
    return mImages.size() - mCurrentImage;
}

int CML::TartanairCapture::imageNumbers() {
    return mImages.size();
}
