#include <sys/stat.h>
#include "cml/capture/TUMCapture.h"

#if CML_HAVE_LIBZIP

#include "cml/map/InternalCalibration.h"



// Inspired by Direct Spare Odometry
CML::TUMCapture::TUMCapture(std::string path) {

    mPath = path;

    loadZip(path + "/images.zip", ".jpg");

    CML_LOG_INFO("Parsing time file...");
    std::ifstream timesFile;
    timesFile.open((path + "/times.txt").c_str());

    while(!timesFile.eof() && timesFile.good())
    {
        std::string line;
        char buf[1000];
        timesFile.getline(buf, 1000);

        int id;
        double stamp;
        float exposure = 0;

        if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure))
        {
            mTimestamps.push_back(stamp);
            mExposures.push_back(exposure);
        }

        else if(2 == sscanf(buf, "%d %lf", &id, &stamp))
        {
            mTimestamps.push_back(stamp);
            mExposures.push_back(exposure);
        }

    }
    timesFile.close();

    if (imageNumbers() != mTimestamps.size()) {
        throw std::runtime_error("The number of files and the number of timestamps are not equal : " + std::to_string(imageNumbers()) + " and " + std::to_string(mTimestamps.size()));
    }

    mGoodExposure = true;
    for (scalar_t exposure : mExposures) {
        if (exposure == 0) {
            mGoodExposure = false;
        }
    }

    CML_LOG_INFO("Parsing groundtruth file...");
    std::ifstream groundtruthFile;
    groundtruthFile.open((path + "/groundtruthSync.txt").c_str());
    while(!groundtruthFile.eof() && groundtruthFile.good())
    {
        char buf[1000];
        groundtruthFile.getline(buf, 1000);

        double stamp;
        double v[7];

        int res = sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf", &stamp, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]);

        if(res == 8)
        {
            Vector3 translation(v[0], v[1], v[2]);
            Eigen::Quaterniond rotation(v[4], v[5], v[6], v[3]);

            mTranslations.emplace_back(translation);
            mRotations.emplace_back(rotation);
        } else {
            CML_LOG_WARN("Invalid line in groundtruthSync.txt : " + std::string(buf) + ". sscanf(...) = " + std::to_string(res));
            for (int i = 0; i < std::min(8, res); i++) {
                CML_LOG_INFO("values[" + std::to_string(i) + "] = " + std::to_string(v[i]));
            }
        }

    }
    groundtruthFile.close();

    std::ifstream lutFile;
    lutFile.open((path + "/pcalib.txt").c_str());
    Vectorf<256> lutValues;
    int lutValuesSize = 0;
    while(!lutFile.eof() && lutFile.good())
    {
        float value; lutFile >> value;
        lutValues[lutValuesSize] = value;
        lutValuesSize++;
        if (lutValuesSize == 256) break;
    }
    lutFile.close();

    if (lutValuesSize == 256) {

        for(int i = 0; i < 256; i++) lutValues[i] = 255.0 * (lutValues[i] - lutValues[0]) / (lutValues[255] - lutValues[0]);

        mLookupTable = GrayLookupTable(lutValues);
    }
/*
    std::cout << "LUT" << std::endl;
    for (int i = 0; i < 256; i++) {
        std::cout << mLookupTable(i) << std::endl;
    }
*/
    if (mTranslations.size() == mTimestamps.size()) {
        CML_LOG_INFO("We found a valid groudtruth");
        mGoodGroundtruth = true;
    } else {
        CML_LOG_ERROR("We found an invalid groudtruth. Found " + std::to_string(mTimestamps.size()) + " timestamps and " + std::to_string(mTranslations.size()) + " cameras.");
        mGoodGroundtruth = false;
    }

    FloatImage image = decompressImage(0).first;
    mWidth = image.getWidth();
    mHeight = image.getHeight();
    mCaptureImageGenerator = new CaptureImageGenerator(mWidth, mHeight);
    mCameraParameters = parseInternalTumCalibration(path + "/camera.txt", mCaptureImageGenerator->getOutputSize());
    CML_LOG_ERROR(mCameraParameters->getPinhole().toString());

    mCurrentIndex = 0;


    mVignette = new FloatImage;
    *mVignette = loadPngImage(mPath + "/vignette.png").first;

    CML_LOG_INFO("TUM Capture " + path + " is open");


}

CML::TUMCapture::~TUMCapture() {
    delete mCaptureImageGenerator;
    delete mCameraParameters;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::TUMCapture::multithreadNext() {

    if (mCurrentIndex >= imageNumbers()) {
        return {};
    }

    Pair<FloatImage, Image> image = decompressImage(mCurrentIndex);
    //image = image.toGrayImage().applyLut(mLookupTable).removeVignette(vignette, vignetteMax).cast<ColorRGB>();

    //image = image.toGrayImage().cast<ColorRGB>();

    //image.saveBmp("luttest.bmp");
    //image.copyToThis(image.horizontalFlip());

    Optional<Camera> groundTruth;
    bool useGroundtruth = mGoodGroundtruth;

    if (mGoodGroundtruth) {
        for (int i = 0; i < 3; i++) useGroundtruth = useGroundtruth && std::isfinite(mTranslations[mCurrentIndex](i));
        for (int i = 0; i < 4; i++) useGroundtruth = useGroundtruth && std::isfinite(mRotations[mCurrentIndex].coeffs()(i));
    }

    if (useGroundtruth) {
        // groundTruth = Camera(-mTranslations[mCurrentIndex], mRotations[mCurrentIndex].matrix().transpose());
        // In CML : The translations and the rotations are inverted
        // Not anymore
        groundTruth = Camera(mTranslations[mCurrentIndex], mRotations[mCurrentIndex].matrix());
    }

    CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
    imageMaker.setImage(image.first)
            .setTime(mTimestamps[mCurrentIndex])
            .setCalibration(mCameraParameters)
            .setLut(&mLookupTable);
    if (mVignette != nullptr) {
        imageMaker.setInverseVignette(*mVignette);
    }

    if (groundTruth.has_value()) {
        imageMaker.setGroundtruth(groundTruth.value());
    }

    if (mGoodExposure) {
        imageMaker.setExposure(mExposures[mCurrentIndex]);
    }

    auto captureImage = imageMaker.generate();
    //(captureImage->getGrayImage(0) * 0.8).horizontalFlip().saveBmp(mExtractedFilePath[mCurrentIndex] + ".bmp");
    mCurrentIndex++;
    return captureImage;
}

int CML::TUMCapture::remaining() {
    return imageNumbers() - mCurrentIndex;
}

int CML::TUMCapture::imageNumbers() {
    return getImageNumber();
}

#endif
