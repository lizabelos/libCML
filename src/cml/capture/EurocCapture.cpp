#include "cml/capture/EurocCapture.h"
#include "cml/map/InternalCalibration.h"

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

CML::EurocCapture::EurocCapture(const std::string &path) {

    std::string line;
    std::ifstream timeFile(path + "/mav0/cam0/data.csv");
    if (timeFile.is_open())
    {
        getline (timeFile,line); // Skip the first line
        while (getline (timeFile,line))
        {
            int pos = line.find(',');
            if (pos < 0) {
                continue;
            }
            mTimes.emplace_back(std::stof(line.substr(0, pos)));
            mImages.emplace_back(path + "/mav0/cam0/data/" + line.substr(pos + 1));
            replace(mImages[mImages.size() - 1], "\n", "");
            replace(mImages[mImages.size() - 1], "\r", "");
        }
        timeFile.close();
    } else {
        throw std::runtime_error("Missing times.txt for Kitty Dataset");
    }

    std::string leftImagePath = mImages[mCurrentImage];
    Image image = loadImage(leftImagePath);

    mCaptureImageGenerator = new CaptureImageGenerator(image.getWidth(), image.getHeight());
    mCalibration = parseInternalEurocCalibration(path + "/mav0/cam0/sensor.yaml");

    mVignette = Array2D<float>(image.getWidth(), image.getHeight(), 1);
    mVignetteMax = 1;

    mIsInit = true;

    logger.info("Euroc dataset " + path + " is open");
}

CML::EurocCapture::~EurocCapture() {
    delete mCaptureImageGenerator;
    delete mCalibration;
}

bool CML::EurocCapture::isInit() {
    return mIsInit;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::EurocCapture::multithreadNext() {
    if (mCurrentImage == mImages.size()) {
        return Ptr<CaptureImage, Nullable>();
    }

    CaptureImageMaker maker = mCaptureImageGenerator->create();


    maker.setImage(loadGrayImage(mImages[mCurrentImage]).cast<float>());

    maker.setPath(mImages[mCurrentImage])
            .setTime(mTimes[mCurrentImage])
            .setCalibration(mCalibration)
            .setLut(&mLut)
            .setInverseVignette(mVignette);

    mCurrentImage++;

    return maker.generate();
}

int CML::EurocCapture::remaining() {
    return mImages[0].size() - mCurrentImage;
}

int CML::EurocCapture::imageNumbers() {
    return mImages[0].size();
}
