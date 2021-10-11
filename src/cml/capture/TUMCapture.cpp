#include <sys/stat.h>
#include "cml/capture/TUMCapture.h"

#if CML_HAVE_LIBZIP

#include "cml/map/InternalCalibration.h"

#define USE_TURBOJPEG 0

#if USE_TURBOJPEG
extern "C" {
    #include <turbojpeg.h>
};
#endif
#include <opencv2/imgcodecs.hpp>


// Inspired by Direct Spare Odometry
CML::TUMCapture::TUMCapture(std::string path, int skipFrame) : mSkipFrame(skipFrame) {

    mPath = path;

    int ziperror = 0;
    mZiparchive = zip_open((path + "/images.zip").c_str(),  ZIP_RDONLY, &ziperror);

    if (ziperror != 0) {
        throw std::runtime_error("Can't open " + path + "/images.zip");
    }

    int numEntries = zip_get_num_entries(mZiparchive, 0);
    logger.info("Found " + std::to_string(numEntries) + " entries");
    for(int k=0;k<numEntries;k++)
    {
        const char* name = zip_get_name(mZiparchive, k,  ZIP_FL_ENC_STRICT);
        std::string nstr = std::string(name);
        if(nstr == "." || nstr == "..") continue;
        mZipFilePath.emplace_back(name);
    }

    std::sort(mZipFilePath.begin(), mZipFilePath.end());

    #ifdef WIN32
    mkdir((path + "/images").c_str());
    #else
    mkdir((path + "/images").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    #endif

    for (const auto& file : mZipFilePath) {
        mExtractedFilePath.emplace_back(path + "/images/" + file);
    }

    logger.info("Parsing time file...");
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

    if (mZipFilePath.size() != mTimestamps.size()) {
        throw std::runtime_error("The number of files and the number of timestamps are not equal");
    }

    mGoodExposure = true;
    for (scalar_t exposure : mExposures) {
        if (exposure == 0) {
            mGoodExposure = false;
        }
    }

    logger.info("Parsing groundtruth file...");
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
            logger.warn("Invalid line in groundtruthSync.txt : " + std::string(buf) + ". sscanf(...) = " + std::to_string(res));
            for (int i = 0; i < std::min(8, res); i++) {
                logger.info("values[" + std::to_string(i) + "] = " + std::to_string(v[i]));
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
        logger.info("We found a valid groudtruth");
        mGoodGroundtruth = true;
    } else {
        logger.error("We found an invalid groudtruth. Found " + std::to_string(mTimestamps.size()) + " timestamps and " + std::to_string(mTranslations.size()) + " cameras.");
        mGoodGroundtruth = false;
    }

    mCameraParameters = parseInternalCalibration(path + "/camera.txt", TUM);
    logger.error(mCameraParameters->getPinhole().toString());

    mCurrentIndex = 0;

    FloatImage image = loadImage(0);

    Vector2 size = mCameraParameters->getOutputSize();

    mWidth = size.x();
    mHeight = size.y();
    mCaptureImageGenerator = new CaptureImageGenerator(mWidth, mHeight);

    {
        cv::Mat m = cv::imread(mPath + "/vignette.png", cv::IMREAD_UNCHANGED);
        if (m.rows == 0 || m.cols == 0) {
            mVignette = nullptr;
        }
        else if (m.type() != CV_16U) {
            throw std::runtime_error("Invalid vignette file format !");
        } else {
            CML::Array2D<uint16_t> vignette16(m.cols, m.rows);
            memcpy(vignette16.data(), m.data, 2 * m.rows * m.cols);
            mVignette = new FloatImage;
            *mVignette = vignette16.cast<float>();
            mVignette->normalize();
            mVignette->elementWiseInverse();
        }
    }

    logger.info("TUM Capture " + path + " is open");


}

CML::TUMCapture::~TUMCapture() {
    delete mCaptureImageGenerator;
    delete mCameraParameters;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::TUMCapture::multithreadNext() {
    while (true) {
        if (mCurrentIndex >= mZipFilePath.size()) {
            return Ptr<CaptureImage, Nullable>(nullptr);
        }

        if (mSkipFrame > 0 && mCurrentIndex % mSkipFrame != 0) {
            mCurrentIndex++;
            continue;
        }

        break;
    }

    if (mCurrentIndex >= mExtractedFilePath.size()) {
        return {};
    }

    FloatImage image = loadImage(mCurrentIndex);
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
    imageMaker.setImage(image)
            .setPath(mExtractedFilePath[mCurrentIndex])
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
    return mZipFilePath.size() - mCurrentIndex;
}


#if USE_TURBOJPEG
CML::FloatImage CML::TUMCapture::loadImage(int id) {

    logger.debug("Extracting and opening " + mExtractedFilePath[id]);

    {
        std::ifstream f(mExtractedFilePath[id]);
        if (!f.good()) {
            logger.warn("TUM is extracting the zip file on the fly, it will be faster on the next execution.");
            extractImage(id);
        }
    }

    {

        FILE * file = fopen(mExtractedFilePath[id].c_str(), "r+");
        if (file == NULL) {
            throw std::runtime_error("Invalid image !");
        }
        fseek(file, 0, SEEK_END);
        long int size = ftell(file);
        rewind(file);
        size = std::max((long)1024*1024, size);
        if (mBufferSize < size) {
            if (mBuffer != nullptr) {
                free(mBuffer);
            }
            mBufferSize = size;
            mBuffer = (unsigned char *)malloc(size);
        }
        int bytes_read = fread(mBuffer, sizeof(unsigned char), mBufferSize, file);
        fclose(file);

        int jpegSubsamp, width, height;

        tjhandle _jpegDecompressor = tjInitDecompress();

        tjDecompressHeader2(_jpegDecompressor, mBuffer, bytes_read, &width, &height, &jpegSubsamp);

        if (mTempImage.getWidth() != width || mTempImage.getHeight() != height) {
            mTempImage = GrayImage(width, height); // todo : preallocate this
        }

        tjDecompress2(_jpegDecompressor, mBuffer, bytes_read, mTempImage.data(), width, 0/*pitch*/, height, TJPF_GRAY, TJFLAG_ACCURATEDCT);

        tjDestroy(_jpegDecompressor);

        return mTempImage.cast<float>();
    }


}
#else

CML::FloatImage CML::TUMCapture::loadImage(int id) {

    logger.debug("Extracting and opening " + mExtractedFilePath[id]);

    {
        std::ifstream f(mExtractedFilePath[id]);
        if (!f.good()) {
            logger.warn("TUM is extracting the zip file on the fly, it will be faster on the next execution.");
            extractImage(id);
        }
    }

    cv::Mat m = cv::imread(mExtractedFilePath[id], cv::IMREAD_GRAYSCALE);
    if(m.rows * m.cols == 0) {
        logger.warn("Invalid image data, trying to extract the image again...");
        std::remove(mExtractedFilePath[id].c_str());
        extractImage(id);
        m = cv::imread(mExtractedFilePath[id], cv::IMREAD_GRAYSCALE);
        if(m.rows * m.cols == 0) {
            throw std::runtime_error("Invalid image !");
        }
    }

    GrayImage image(m.cols, m.rows);
    memcpy(image.data(), m.data, m.rows * m.cols);
    return image.cast<float>();
}

#endif

void CML::TUMCapture::extractImage(int id) {


    mZipBuffer.resize(8192);

    zip_file_t *zipFile = zip_fopen(mZiparchive, mZipFilePath[id].c_str(), 0);
    std::ofstream fout(mExtractedFilePath[id], std::ios::out | std::ios::binary);

    long readBytes = 0;

    while (true) {

        readBytes = zip_fread(zipFile, mZipBuffer.data(), mZipBuffer.size());
        if (readBytes <= 0) {
            break;
        }

        fout.write(mZipBuffer.data(), readBytes);

    }

    zip_fclose(zipFile);
    fout.close();


}

int CML::TUMCapture::imageNumbers() {
    return mImages.size();
}

#endif
