#ifndef CML_ZIPTIFFCAPTURE_H
#define CML_ZIPTIFFCAPTURE_H

#include "cml/config.h"

#if CML_HAVE_LIBZIP

#include "ZipCaptureHelper.h"


namespace CML {

    class ZipTiffCapture : public AbstractMultithreadFiniteCapture, public ZipCaptureHelper {

    public:
        inline ZipTiffCapture(const std::string &zipPath) {
            logger.important("open zip file");
            loadZip(zipPath, ".tif");

            uint8_t *data;
            size_t size;
            decompressFile(1, &data, &size);
            logger.important("load tiff image");
            auto images = loadTiffImage(data, size);
            logger.important("load mask");
            mMask = loadPngImage(zipPath + ".mask.png").first.castToUChar<unsigned char>();
            logger.important("mask loaded");
            mMask.saveBmp("/home/tdaumain/mask.bmp");

            //Add the real Time of the images
            // Parse Time file
            logger.important("Parsing time file...");
            std::ifstream timesFile;
            timesFile.open((zipPath + ".times.txt").c_str());
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


            int histogram[256];
            for (int i = 0; i < 256; i++) {
                histogram[i] = 0;
            }
            for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    histogram[(int)images.first(x,y)]+=1;
                }
            }
            int threshold = (images.first.getWidth() * images.first.getHeight()) * 0.999;
            for (int i = 1; i < 256; i++) {
                histogram[i] += histogram[i - 1];
                if (histogram[i] > threshold) {
                    mImageMax = i;
                    break;
                }
            }

            mLookupTable = GrayLookupTable::exp(255, 1.005f);

            images.first = images.first * (255.0f / mImageMax);

            //crop with mask
            int top = 0, bottom = images.first.getHeight() - 1;

            for (int y = 0; y < images.first.getHeight(); y++) {
                for (int x = 0; x < images.first.getWidth(); x++) {
                    if (mMask(x,y) < 128 || images.first(x,y) > 255.9) {
                        if (y < images.first.getHeight() / 2) {
                            top = std::max(top, y);
                        } else {
                            bottom = std::min(bottom, y);
                        }
                    }
                }
            }
            logger.important("TOP = " + std::to_string(top) + " BOTTOM = " + std::to_string(bottom));


            logger.important("create captureimagegenrator");
            mCaptureImageGenerator = new CaptureImageGenerator(images.first.getWidth(), (bottom - top));

            logger.important("Internal Calibration");
            mCameraParameters = parseInternalStereopolisCalibration(zipPath + ".xml", mCaptureImageGenerator->getOutputSize(), top, bottom);


            logger.important("ZipTiffCapture created");

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
            decompressFile(mCurrentImage, &data, &size);
            auto images = loadTiffImage(data, size);

            images.first = images.first * (255.0f / mImageMax);

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
                    .setPath(getFilename(mCurrentImage))
                    /*.setTime((scalar_t)mCurrentImage / 10.0)*/.setTime(mTimestamps[mCurrentImage])
                    .setCalibration(mCameraParameters)
                    .setLut(&mLookupTable);

            mCurrentImage++;

            return imageMaker.generate();
        }

    private:
        CaptureImageGenerator *mCaptureImageGenerator;
        InternalCalibration *mCameraParameters;
        int mCurrentImage = 1;
        GrayImage mMask;
        GrayLookupTable mLookupTable;
        float mImageMax = 0;
        std::vector<scalar_t> mTimestamps;


    };

}

#endif

#endif
