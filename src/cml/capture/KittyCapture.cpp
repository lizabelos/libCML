//
// Created by tbelos on 16/05/19.
//
#include <fstream>

#include "cml/capture/KittyCapture.h"
#include "cml/map/InternalCalibration.h"

#include <sys/stat.h>

CML::KittyCapture::KittyCapture(const std::string &path, bool useColor) : mUseColor(useColor) {

    std::size_t found = path.find_last_of("/\\");
    std::string id = path.substr(found+1);

    struct stat info;
    if( stat( (path + "/image_0").c_str(), &info ) != 0 ) {
        throw std::runtime_error("Missing " + path + "/image_0");
    }
    else if( info.st_mode & S_IFDIR ) {
        // OK
    }
    else {
        throw std::runtime_error(path + "/images_0 is not a directory");
    }

    std::string line;
    std::ifstream timeFile(path + "/times.txt");
    if (timeFile.is_open())
    {
        while (getline (timeFile,line))
        {
            mTimes.emplace_back(std::stof(line));
        }
        timeFile.close();
    } else {
        throw std::runtime_error("Missing times.txt for Kitty Dataset");
    }

    std::ifstream posesFile(path + "/../../poses/" + id + ".txt");
    if (posesFile.is_open()) {

        while (getline (posesFile,line))
        {
            std::vector<std::string> values;
            split(line, values, ' ');

            KittyPose pose;
            pose.transformMatrix = Matrix44::Identity();
            for (size_t i = 0; i< values.size(); i++) {
                pose.transformMatrix(i / 4, i % 4) = std::stod(values[i].c_str());
            }

            mPoses.emplace_back(pose);

        }
        timeFile.close();

        posesFile.close();
    } else {
        //throw std::runtime_error("Missing " + path + "/../../poses/" + id + ".txt" + " for Kitty Dataset");
    }

    mImages[0].resize(mTimes.size());
    mImages[1].resize(mTimes.size());
    mImages[2].resize(mTimes.size());
    mImages[3].resize(mTimes.size());
    for (size_t i = 0; i < mTimes.size(); i++) {
        std::string filename = std::to_string(i);
        while (filename.size() < 6) filename = "0" + filename;
        filename = filename + ".png";
        mImages[0][i] = path + "/image_0/" + filename;
        mImages[1][i] = path + "/image_1/" + filename;
        mImages[2][i] = path + "/image_2/" + filename;
        mImages[3][i] = path + "/image_3/" + filename;
    }

    std::string leftImagePath = mImages[0][mCurrentImage];
    Image image = loadPngImage(leftImagePath).second;

    mVignette = Array2D<float>(image.getWidth(), image.getHeight(), 1.0f);
    mVignetteMax = 1;

    std::ifstream calibFile(path + "/calib.txt");
    if (calibFile.is_open())
    {
        int i = 0;
        double w = image.getWidth();
        double h = image.getHeight();
        Vector2 s(w, h);
        while (getline (calibFile,line))
        {
            double ic[12];
            int n = std::sscanf(&line.c_str()[3], "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7], &ic[8], &ic[9], &ic[10], &ic[11]);
            if (n != 12) {
                throw std::runtime_error("Malformed calib.txt for Kitty Dataset");
            }

            double fx = ic[0];
            double cx = ic[2] - 0.5;
            double fy = ic[5];
            double cy = ic[6] - 0.5;

            PinholeUndistorter pinhole(Vector2(fx, fy), Vector2(cx, cy));

            mCalibration[i] = new InternalCalibration(pinhole, s);

            i++;
            if (i == 4) {
                break;
            }
        }
        if (i != 4) {
            throw std::runtime_error("Malformed calib.txt for Kitty Dataset");
        }
        calibFile.close();
    } else {
        throw std::runtime_error("Missing calib.txt for Kitty Dataset");
    }

    mCaptureImageGenerator = new CaptureImageGenerator(image.getWidth(), image.getHeight());
    CML_LOG_INFO("Kitty Dataset " + id + " at '" + path + "' is open");

    //mLut = GrayLookupTable::exp(255, 1.005f);

}

CML::KittyCapture::~KittyCapture() {
    delete mCaptureImageGenerator;
    delete mCalibration[0];
    delete mCalibration[1];
}

bool CML::KittyCapture::isInit() {
    return mIsInit;
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::KittyCapture::multithreadNext() {
    if (mCurrentImage == mImages[0].size()) {
        return Ptr<CaptureImage, Nullable>();
    }

    int currentImage = mCurrentImage;
    if (isReverse()) {
        currentImage = (mImages[0].size() - 1) - currentImage;
    }

    CaptureImageMaker maker = mCaptureImageGenerator->create();

    if (mUseColor) {
        auto images = loadPngImage(mImages[2][currentImage]);
        maker.setImage(images.first);
        //maker.setImage(images.second);
    } else {
        auto images = loadPngImage(mImages[0][currentImage]);
        maker.setImage(images.first);
        //maker.setImage(images.second);
    }

    maker.setPath(mImages[0][currentImage])
            .setTime(mTimes[mCurrentImage])
            .setCalibration(mCalibration[0])
            .setLut(&mLut)
            .setInverseVignette(mVignette);

    if (mPoses.size() > 0) {
        maker.setGroundtruth(Camera::fromNullspaceMatrix(mPoses[currentImage].transformMatrix));
    }

    mCurrentImage++;

    return maker.generate();


}

int CML::KittyCapture::remaining() {
    return mImages[0].size() - mCurrentImage;
}

int CML::KittyCapture::imageNumbers() {
    return mImages[0].size();
}
