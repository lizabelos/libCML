#include <sys/stat.h>
#include "cml/capture/StereopolisCapture.h"

#if CML_HAVE_LIBZIP

#include "cml/map/InternalCalibration.h"

#define USE_TURBOJPEG 0

#if USE_TURBOJPEG
extern "C" {
    #include <turbojpeg.h>
};
#endif
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tiffio.h>
#include <filesystem>

void CML::StereopolisCapture::createPathList(std::string path){
    for (const auto & entry : std::filesystem::directory_iterator(path))
        if(entry.path().extension() == ".tif" ) mPathList.push_back(entry.path());
    std::sort(mPathList.begin(), mPathList.end());
}

void CML::StereopolisCapture::createPathListAll(const std::string & path){
    std::string s1 = path + "/section_00";
    std::string s2 = path + "/section_01";
    for (const auto & entry : std::filesystem::directory_iterator(s1))
        if(entry.path().extension() == ".tif" ) mPathList.push_back(entry.path());
    for (const auto & entry : std::filesystem::directory_iterator(s2))
        if(entry.path().extension() == ".tif" ) mPathList.push_back(entry.path());
    std::sort(mPathList.begin(), mPathList.end());
    if(mReverse)
      std::reverse(mPathList.begin(), mPathList.end());
    if(mStart > 0)
      mPathList.erase(mPathList.begin(), mPathList.begin() + mStart);
}

CML::StereopolisCapture::StereopolisCapture(const std::string& path, const bool& reverse, const int& start, const double& expFactor, const int& topFactor ){
    logger.important("create stereopolis capture");

    mStart = start;
    mReverse = reverse;

    //createPathList(path);
    createPathListAll(path);

    std::string s = "number of files : " + std::to_string(mPathList.size());
    logger.important(s);

    // Parse Time file
    logger.important("Parsing time file...");
    std::ifstream timesFile;
    timesFile.open((path + "/times.txt").c_str());
    if (timesFile.is_open())
    {
    		while(!timesFile.eof() && timesFile.good())
    		{
    		    char buf[1000];
    		    timesFile.getline(buf, 1000);
    		    int id;
    		    double stamp;
    		    if(2 == sscanf(buf, "%d %lf", &id, &stamp))
    		    {
    		        mTimestamps.push_back(stamp);
    		    }

        }
        timesFile.close();
    } else {
        throw std::runtime_error("Missing times.txt for STEREOPOLIS Dataset");
    }


    if(reverse){
      std::reverse(mTimestamps.begin(), mTimestamps.end());
      logger.raw(std::to_string(mTimestamps[0]) + " " +std::to_string(mTimestamps[mTimestamps.size()]));
      double d = mTimestamps[0];
      for (size_t i = 0; i < mTimestamps.size(); i++) {
        mTimestamps[i] = d - mTimestamps[i];
        if(mTimestamps[i] < 0) throw std::runtime_error("bug in timestamp init");
      }
    }

    if(start > 0){
      mTimestamps.erase(mTimestamps.begin(), mTimestamps.begin() + start);
    }


    logger.important("load tiff image");
    auto images = loadTiffImage(mPathList[0]);
    logger.important("create mask");
    mMask = loadPngImage(path + "/mask.png").first.castToUChar<unsigned char>();

    mLookupTable = GrayLookupTable::exp(255, expFactor); // add at execution with -e (default= 1.005)

    int top = topFactor, bottom = images.first.getHeight() - 1; // add at execution with -k (default = 1000)
    // int top = 436, bottom = 1275;

    for (int y = 0; y < images.first.getHeight(); y++) {
        for (int x = 0; x < images.first.getWidth(); x++) {
          if (mMask(x,y) < 128) {
                if (y < images.first.getHeight() / 2) {
                    top = std::max(top, y);
                } else {
                    bottom = std::min(bottom, y);
                }
            }
        }
    }
    logger.important("TOP = " + std::to_string(top) + " BOTTOM = " + std::to_string(bottom));

    logger.important("create capture image");
    mCaptureImageGenerator = new CaptureImageGenerator(images.first.getWidth(), (bottom - top));


    logger.important("create calib");
    mCameraParameters = parseInternalStereopolisCalibration(path + "/calib.xml", mCaptureImageGenerator->getOutputSize(), top, bottom);

    mCurrentIndex = 0;

    logger.info("STEREOPOLIS Capture " + path + " is open");

}

CML::StereopolisCapture::~StereopolisCapture() {
    delete mCaptureImageGenerator;
    delete mCameraParameters;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::StereopolisCapture::multithreadNext() {
    if (mCurrentIndex >= mPathList.size()) {
        return Ptr<CaptureImage, Nullable>(nullptr);
    }
    auto images = loadTiffImage(mPathList[mCurrentIndex]);

    // images.first = images.first * (255.0f / mImageMax);
    for (int y = 0; y < images.first.getHeight(); y++) {
        for (int x = 0; x < images.first.getWidth(); x++) {
            if (mMask(x,y) < 128) {
                images.first(x,y) = std::numeric_limits<float>::quiet_NaN();
                images.second(x,y) = ColorRGBA(0,0,0,0);
            }
        }
    }
    CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
                imageMaker.setImage(images.first)
                        .setImage(images.second)
                        .setPath(mPathList[mCurrentIndex])
                        .setTime(mTimestamps[mCurrentIndex])
                        .setCalibration(mCameraParameters)
                        .setLut(&mLookupTable)
                        ;

    mCurrentIndex++;

    return imageMaker.generate();
}

int CML::StereopolisCapture::remaining() {
    return mPathList.size() - mCurrentIndex;
}

#endif
