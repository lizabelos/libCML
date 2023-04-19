#ifndef CML_ZIPSTEREOPOLISCAPTURE_H
#define CML_ZIPSTEREOPOLISCAPTURE_H

#include "cml/config.h"

#if CML_HAVE_LIBZIP

#include "ZipCaptureHelper.h"
#include "cml/image/Filter.h"

namespace CML {

    class StereopolisPose {

    public:
        float time;
        Matrix44 transformMatrix;
    };


    class ZipStereopolisCapture : public AbstractMultithreadFiniteCapture, public ZipCaptureHelper {

    public:
        inline ZipStereopolisCapture(const std::string &zipPath) {

            std::string extractPath = zipPath + "_uncompressed";
            if (!std::filesystem::is_directory(extractPath) || !std::filesystem::exists(extractPath)) { // Check if src folder exists
                std::filesystem::create_directory(extractPath);
            }
            loadZip(zipPath, "_0.tif", extractPath);

            uint8_t *data;
            size_t size;
            decompressFile(1, &data, &size);
            auto images = loadTiffImage(data, size);
            //mMask = loadPngImage(zipPath + ".mask.png").first.castToUChar<unsigned char>();
            mLookupTable = GrayLookupTable::gamma(2.2 / 1.8);

            const int mWidthCrop = 20;
            mCropOrigin = {mWidthCrop / 2, images.second.getHeight() / 4 - 10};
            mCropSize = {images.second.getWidth() - mWidthCrop, images.second.getHeight() / 2};


            mCaptureImageGenerator = new CaptureImageGenerator(mCropSize.x(), mCropSize.y());

            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml", mCaptureImageGenerator->getOutputSize(), mCropOrigin, mCropSize);


            std::ifstream timesFile(zipPath + ".times.txt");
            if (timesFile.is_open()) {
                std::string line;
                while (getline (timesFile,line))
                {
                    std::vector<std::string> values;
                    if (line.empty()) {
                        continue;
                    }
                    split(line, values, ' ');

                    mTimes.emplace_back(std::stod(values[1].c_str()));

                }

                timesFile.close();
            } else {
                throw std::runtime_error("Missing times");
            }

/*            std::ifstream posesFile(zipPath + ".gt.txt");
            if (posesFile.is_open()) {

                std::string line;
                while (getline (posesFile,line))
                {
                    std::vector<std::string> values;
                    split(line, values, ' ');

                    StereopolisPose pose;
                    pose.time = std::stod(values[0].c_str());
                    pose.transformMatrix = Matrix44::Identity();
                    for (size_t i = 1; i< values.size(); i++) {
                        pose.transformMatrix((i - 1) / 4, (i - 1) % 4) = std::stod(values[i].c_str());
                    }

                    mPoses.emplace_back(pose);

                }

                posesFile.close();
            }

*/
        }

        inline int remaining() final {
            return imageNumbers() - mCurrentImage;
        }

        inline int imageNumbers() final {
            return getImageNumber();
        }

    protected:
        inline Ptr<CaptureImage, Nullable> multithreadNext() {
            uint8_t *data;
            size_t size;
            std::string decompressedFilePath = decompressFile(mCurrentImage, &data, &size);
            auto images = loadTiffImage(data, size);


            /*for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    if (mMask(x,y) < 128 || (y < images.first.getHeight() / 3 && images.first(x,y) > 250)) {
                        images.first(x,y) = std::numeric_limits<float>::quiet_NaN();
                        images.second(x,y) = ColorRGBA(0,0,0,0);
                    }
                }
            }*/
            images.first = images.first.crop(mCropOrigin.x(), mCropOrigin.y(), mCropSize.x(), mCropSize.y());
            images.second = images.second.crop(mCropOrigin.x(), mCropOrigin.y(), mCropSize.x(), mCropSize.y());

           // images.first = images.first.convertGamma(1.8f, 2.2f);

            CaptureImageMaker imageMaker = mCaptureImageGenerator->create();
            imageMaker.setImage(images.first)
              //      .setImage(images.second)
                    .setPath(decompressedFilePath)
                    .setTime((scalar_t)mCurrentImage / 10.0)
                    .setCalibration(mCameraParameters)
                    .setLut(&mLookupTable);

            if (mPoses.size() > 0) {

                float currentTime = mTimes[mCurrentImage];

                int poseId = 0;
                float bestDistance = std::numeric_limits<float>::max();
                for (int i = 0; i < mPoses.size(); i++) {
                    if (abs(currentTime - mPoses[i].time) < bestDistance) {
                        bestDistance = abs(currentTime - mPoses[poseId].time);
                        poseId = i;
                    }
                }

                imageMaker.setGroundtruth(Camera::fromNullspaceMatrix(mPoses[poseId].transformMatrix));
            }

            mCurrentImage++;

            return imageMaker.generate();
        }

    private:
        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        GrayLookupTable mLookupTable;
        int mCurrentImage = 1;
        //GrayImage mMask;
        List<StereopolisPose> mPoses;
        List<float> mTimes;

        Vector2i mCropOrigin, mCropSize;

    };

}

#endif

#endif