#include "cml/capture/Eth3DCapture.h"
#include "cml/map/InternalCalibration.h"

CML::Eth3DCapture::Eth3DCapture(const std::string &path) {
    mImages = listDirectory(path + "/rgb", "png");
    if (mImages.size() == 0) {
        throw std::runtime_error("No images");
    }
    mTimes.resize(mImages.size());
    std::sort(mImages.begin(), mImages.end());
    for (size_t i = 0; i < mImages.size(); i++) {
        mTimes[i] = (float)i / 30.0;
    }

    std::string leftImagePath = mImages[mCurrentImage];
    Image image = loadImage(leftImagePath);

    mVignette = Array2D<float>(image.getWidth(), image.getHeight(), 1);
    float w = image.getWidth();
    float h = image.getHeight();
    mVignetteMax = 1;
    mCaptureImageGenerator = new CaptureImageGenerator(image.getWidth(), image.getHeight());

    std::ifstream calibFile(path + "/calibration.txt");
    if (calibFile.is_open())
    {
        std::string line;
        getline (calibFile, line);
        double ic[4];
        int n = std::sscanf(&line.c_str()[3], "%lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3]);
        if (n != 4) {
            throw std::runtime_error("Malformed calibration.txt for Kitty Dataset");
        }
        mCalibration = new InternalCalibration(PinholeUndistorter(Vector2(ic[0], ic[1]), Vector2(ic[2], ic[3])), Vector2(w, h));
    } else {
        throw std::runtime_error("Missing calibration.txt for Kitty Dataset");
    }

    mIsInit = true;
}

CML::Eth3DCapture::~Eth3DCapture() {

}

bool CML::Eth3DCapture::isInit() {
    return mIsInit;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::Eth3DCapture::multithreadNext() {
    if (mCurrentImage == mImages.size()) {
        return Ptr<CaptureImage, Nullable>();
    }

    CaptureImageMaker maker = mCaptureImageGenerator->create();

    maker.setImage(loadImage(mImages[mCurrentImage]));

    maker.setPath(mImages[mCurrentImage])
            .setTime(mTimes[mCurrentImage])
            .setCalibration(mCalibration)
            .setLut(&mLut)
            .setInverseVignette(mVignette);

    mCurrentImage++;

    return maker.generate();
}

int CML::Eth3DCapture::remaining() {
    return AbstractCapture::remaining();
}

int CML::Eth3DCapture::imageNumbers() {
    return mImages.size();
}
