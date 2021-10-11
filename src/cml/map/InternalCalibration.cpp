//
// Created by tbelos on 19/04/19.
//

#include <stdexcept>
#include "cml/map/InternalCalibration.h"
#include <yaml-cpp/yaml.h>

#ifdef WITH_OPENMP
#include <omp.h>
#endif

// From OSBRLAM
namespace CML {
    Pair<PinholeUndistorter, Array2D<Vector2>> makeOptimalK_crop(PinholeUndistorter originalPinhole, Undistorter *preundistorter, Vector2i originalSize, Vector2i newSize) {
        Array2D<Vector2> undistortMap(newSize.x(), newSize.y());
        float remapX[newSize.x() * newSize.y()], remapY[newSize.x() * newSize.y()];

        // 1. stretch the center lines as far as possible, to get initial coarse quess.
        float *tgX = new float[100000];
        float *tgY = new float[100000];
        float minX = 0;
        float maxX = 0;
        float minY = 0;
        float maxY = 0;

        for (int x = 0; x < 100000; x++) {
            tgX[x] = (x - 50000.0f) / 10000.0f;
            tgY[x] = 0;
        }
        for (int i = 0; i < 100000; i++) {
            Vector2 output = originalPinhole.distort(preundistorter->distort(Vector2(tgX[i], tgY[i])));
            tgX[i] = output.x();
            tgY[i] = output.y();
        }
        for (int x = 0; x < 100000; x++) {
            if (tgX[x] > 0 && tgX[x] < originalSize.x() - 1) {
                if (minX == 0) minX = (x - 50000.0f) / 10000.0f;
                maxX = (x - 50000.0f) / 10000.0f;
            }
        }
        for (int y = 0; y < 100000; y++) {
            tgY[y] = (y - 50000.0f) / 10000.0f;
            tgX[y] = 0;
        }
        for (int i = 0; i < 100000; i++) {
            Vector2 output = originalPinhole.distort(preundistorter->distort(Vector2(tgX[i], tgY[i])));
            tgX[i] = output.x();
            tgY[i] = output.y();
        }
        for (int y = 0; y < 100000; y++) {
            if (tgY[y] > 0 && tgY[y] < originalSize.y() - 1) {
                if (minY == 0) minY = (y - 50000.0f) / 10000.0f;
                maxY = (y - 50000.0f) / 10000.0f;
            }
        }
        delete[] tgX;
        delete[] tgY;

        minX *= 1.01;
        maxX *= 1.01;
        minY *= 1.01;
        maxY *= 1.01;


        printf("initial range: x: %.4f - %.4f; y: %.4f - %.4f!\n", minX, maxX, minY, maxY);



        // 2. while there are invalid pixels at the border: shrink square at the side that has invalid pixels,
        // if several to choose from, shrink the wider dimension.
        bool oobLeft = true, oobRight = true, oobTop = true, oobBottom = true;
        int iteration = 0;
        while (oobLeft || oobRight || oobTop || oobBottom) {
            oobLeft = oobRight = oobTop = oobBottom = false;
            for (int y = 0; y < newSize.y(); y++) {
                remapX[y * 2] = minX;
                remapX[y * 2 + 1] = maxX;
                remapY[y * 2] = remapY[y * 2 + 1] = minY + (maxY - minY) * (float) y / ((float) newSize.y() - 1.0f);
            }
            for (int i = 0; i < 2 * newSize.y(); i++) {
                Vector2 output = originalPinhole.distort(preundistorter->distort(Vector2(remapX[i], remapY[i])));
                remapX[i] = output.x();
                remapY[i] = output.y();
            }
            for (int y = 0; y < newSize.y(); y++) {
                if (!(remapX[2 * y] > 0 && remapX[2 * y] < originalSize.x() - 1))
                    oobLeft = true;
                if (!(remapX[2 * y + 1] > 0 && remapX[2 * y + 1] < originalSize.x() - 1))
                    oobRight = true;
            }


            for (int x = 0; x < newSize.x(); x++) {
                remapY[x * 2] = minY;
                remapY[x * 2 + 1] = maxY;
                remapX[x * 2] = remapX[x * 2 + 1] = minX + (maxX - minX) * (float) x / ((float) newSize.x() - 1.0f);
            }
            for (int i = 0; i < 2 * newSize.x(); i++) {
                Vector2 output = originalPinhole.distort(preundistorter->distort(Vector2(remapX[i], remapY[i])));
                remapX[i] = output.x();
                remapY[i] = output.y();
            }

            for (int x = 0; x < newSize.x(); x++) {
                if (!(remapY[2 * x] > 0 && remapY[2 * x] < originalSize.y() - 1))
                    oobTop = true;
                if (!(remapY[2 * x + 1] > 0 && remapY[2 * x + 1] < originalSize.y() - 1))
                    oobBottom = true;
            }


            if ((oobLeft || oobRight) && (oobTop || oobBottom)) {
                if ((maxX - minX) > (maxY - minY))
                    oobBottom = oobTop = false;    // only shrink left/right
                else
                    oobLeft = oobRight = false; // only shrink top/bottom
            }

            if (oobLeft) minX *= 0.995;
            if (oobRight) maxX *= 0.995;
            if (oobTop) minY *= 0.995;
            if (oobBottom) maxY *= 0.995;

            iteration++;


            printf("iteration %05d: range: x: %.4f - %.4f; y: %.4f - %.4f!\n", iteration, minX, maxX, minY, maxY);
            if (iteration > 500) {
                printf("FAILED TO COMPUTE GOOD CAMERA MATRIX - SOMETHING IS SERIOUSLY WRONG. ABORTING \n");
                exit(1);
            }
        }

        for (int i = 0; i < newSize.x() * newSize.y(); i++) {
            undistortMap.data()[i] = Vector2(remapX[i], remapY[i]);
        }

        float fx = ((float) newSize.x() - 1.0f) / (maxX - minX);
        float fy = ((float) newSize.y() - 1.0f) / (maxY - minY);

        return Pair<PinholeUndistorter, Array2D<Vector2>>(
                PinholeUndistorter(
                        Vector2(fx,fy),
                        Vector2((-minX * fx),(-minY * fy))
                ),
                undistortMap
                );

    }
}

CML::InternalCalibration* CML::parseInternalTumCalibration(std::string path) {
    std::ifstream f(path.c_str());
    logger << "Reading Photometric Calibration from file " << path << endl;
    if (!f.good())
    {
        throw std::runtime_error("Couldn't open " + path + "'");
    }

    std::string line[4];
    for (int i = 0; i < 4; i++) {
        std::getline(f, line[i]);
    }

    PinholeUndistorter originalPinhole;
    Undistorter *preundistorter = nullptr;
    PinholeUndistorter newPinhole;
    Vector2 originalSize = Vector2(0, 0);
    Vector2 newSize = Vector2(0, 0);


    // Read first line : camera original parameters
    double ic[9];

    int i = std::sscanf(line[0].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]);

    if (i == 4) {
        logger.info("Using PinHole only");
        originalPinhole = PinholeUndistorter(Vector2(ic[0], ic[1]), Vector2(ic[2], ic[3]));
    }
    else if (i == 5) {
        logger.info("Using PinHole + FOV");
        originalPinhole = PinholeUndistorter(Vector2(ic[0], ic[1]), Vector2(ic[2], ic[3]));
        preundistorter = new FOVUndistorter(ic[4]);
    }
    else if(i == 8)
    {
        logger.info("Using PinHole + Radtan");
        originalPinhole = PinholeUndistorter(Vector2(ic[0], ic[1]), Vector2(ic[2], ic[3]));
        preundistorter = new RadtanUndistorter(ic[4], ic[5], ic[6], ic[7]);
    } else {
        throw std::runtime_error("Unknown distortion model :(");
    }

    // Read second line : original size
    i = std::sscanf(line[1].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]);

    if (i == 2) {
        originalSize = Vector2(ic[0], ic[1]);
        // originalPinhole = originalPinhole.recenter(originalSize); // todo : do we need to recenter the pinhole here ? to try
    } else {
        throw std::runtime_error("Malformed file :(");
    }

    originalPinhole = originalPinhole.scaleAndRecenter(originalSize, Vector2(-0.5, -0.5));

    Array2D<Vector2> undistortMap;

    // Read the third line : new camera parameters
    if (line[2] == "crop") {

        // Read fourth line : new size
        i = std::sscanf(line[3].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]);

        if (i == 2) {
            newSize = Vector2(ic[0], ic[1]);
        } else {
            throw std::runtime_error("Malformed file :(");
        }

        auto res = makeOptimalK_crop(originalPinhole, preundistorter, originalSize.cast<int>(), newSize.cast<int>());
        newPinhole = res.first;
        undistortMap = res.second;

        //  newPinhole = PinholeUndistorter(Vector2(277.34 / 640.0, 291.402 / 480.0), Vector2(312.234 / 640.0, 239.777 / 480.0));

    } else {

        i = std::sscanf(line[2].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4],
                        &ic[5], &ic[6], &ic[7]);

        if (i == 5) {
            newPinhole = PinholeUndistorter(Vector2(ic[0], ic[1]), Vector2(ic[2], ic[3]));
        } else {
            throw std::runtime_error("Malformed file :(");
        }

        // Read fourth line : new size
        i = std::sscanf(line[3].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]);

        if (i == 2) {
            newSize = Vector2(ic[0], ic[1]);
        } else {
            throw std::runtime_error("Malformed file :(");
        }

        newPinhole = newPinhole.scaleAndRecenter(newSize, Vector2(-0.5, -0.5));

    }

    InternalCalibration *result;
    //if (!haveUndistortMap) {

    result = new InternalCalibration(originalPinhole, originalSize, preundistorter, newPinhole, newSize);
    //} else {
    //    result = new InternalCalibration(originalPinhole, originalSize, preundistorter, newPinhole, newSize, undistortMap);
    //}
    for (int lvl = 0; lvl < 3; lvl++) {
        logger.info(result->toString(lvl));
    }
    return result;
}

CML::InternalCalibration* CML::parseInternalEurocCalibration(std::string path) {
    std::cout << "Parsing calibration file : " << path << std::endl;

    YAML::Node root = YAML::LoadFile(path);

    int width = root["resolution"][0].as<int>();
    int height = root["resolution"][1].as<int>();

    std::string cameraModel = root["camera_model"].as<std::string>();

    scalar_t fx = root["intrinsics"][0].as<scalar_t>();
    scalar_t fy = root["intrinsics"][1].as<scalar_t>();
    scalar_t cx = root["intrinsics"][2].as<scalar_t>();
    scalar_t cy = root["intrinsics"][3].as<scalar_t>();

    PinholeUndistorter pinhole(Vector2(fx, fy), Vector2(cx, cy));
    pinhole.scaleAndRecenter(Vector2(0,0), Vector2(-0.5, -0.5));

    Undistorter *undistorter = nullptr;

    std::string distortionModel = root["distortion_model"].as<std::string>();

    if (distortionModel == "radial-tangential") {
        float coeffs[4];
        for (int i = 0; i < 4; i++) coeffs[i] = root["distortion_coefficients"][i].as<float>();
        undistorter = new RadtanUndistorter(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
    }
    else if (distortionModel != "") {
        throw std::runtime_error("Invalid distortion model : " + distortionModel);
    }

    auto res = makeOptimalK_crop(pinhole, undistorter, Vector2i(width, height), Vector2i(width, height));

    return new InternalCalibration(pinhole, Vector2(width, height), undistorter, res.first, Vector2(width, height));

}

CML::InternalCalibration* CML::parseInternalCalibration(std::string path, CML::InternalCalibrationFileFormat format) {

    setlocale(LC_ALL, "C");

    switch (format) {
        case TUM:
            return parseInternalTumCalibration(path);
        case EUROC:
            return parseInternalEurocCalibration(path);

    }

    throw std::runtime_error("Unknown distortion file format :(");

}

void CML::InternalCalibration::computeUndistortMap() {

    mUndistortMap = Array2D<Vector2f>(mNewSize.x(), mNewSize.y());

    for (int y = 0; y < mNewSize.y(); y++) {
         for (int x = 0; x < mNewSize.x(); x++) {

            Vector2f newPosition(x, y);


            Vector2f originalPosition;
            if (mPreundistorter != nullptr) {
                originalPosition = mNewPinhole.undistort(newPosition);
                originalPosition = mPreundistorter->distort(originalPosition);
                originalPosition = mOriginalPinhole.distort(originalPosition);
            } else {
                originalPosition = mNewPinhole.undistort(newPosition);
                originalPosition = mOriginalPinhole.distort(originalPosition);
            }

            if (originalPosition.x() < 0 || originalPosition.y() < 0 || originalPosition.x() >= mOriginalSize.x() - 1 || originalPosition.y() >= mOriginalSize.y() - 1) {
                mUndistortMap(x, y) = Vector2f(NAN, NAN);
            } else {
                mUndistortMap(x, y) = originalPosition;
            }

        }
    }

}