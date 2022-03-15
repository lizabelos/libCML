//
// Created by tbelos on 19/04/19.
//

#include <stdexcept>
#include "cml/map/InternalCalibration.h"
#if CML_HAVE_YAML_CPP
#include <yaml-cpp/yaml.h>
#endif
#include "rapidxml/rapidxml.hpp"

#ifdef WITH_OPENMP
#include <omp.h>
#endif

// From OSBRLAM
namespace CML {
    Pair<PinholeUndistorter, Array2D<Vector2f>> makeOptimalK_crop(PinholeUndistorter originalPinhole, Undistorter *preundistorter, Vector2i originalSize, Vector2i newSize) {
        Array2D<Vector2f> undistortMap(newSize.x(), newSize.y());
        float *remapX = new float[newSize.x() * newSize.y()];
        float *remapY = new float[newSize.x() * newSize.y()];

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
            undistortMap.data()[i] = Vector2f(remapX[i], remapY[i]);
        }

        float fx = ((float) newSize.x() - 1.0f) / (maxX - minX);
        float fy = ((float) newSize.y() - 1.0f) / (maxY - minY);

        delete[] remapX;
        delete[] remapY;

        return Pair<PinholeUndistorter, Array2D<Vector2f>>(
                PinholeUndistorter(
                        Vector2(fx,fy),
                        Vector2((-minX * fx),(-minY * fy))
                ),
                undistortMap
                );

    }
}

CML::InternalCalibration* CML::parseInternalTumCalibration(std::string path, Vector2i outputSize) {
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
    const Vector2 newSize = outputSize.cast<scalar_t>();


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

    Array2D<Vector2f> undistortMap;

    // Read the third line : new camera parameters
    if (line[2] == "crop") {

        // Read fourth line : new size
        i = std::sscanf(line[3].c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &ic[0], &ic[1], &ic[2], &ic[3], &ic[4], &ic[5], &ic[6], &ic[7]);

        if (i == 2) {
            //newSize = Vector2(ic[0], ic[1]);
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
            //newSize = Vector2(ic[0], ic[1]);
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

CML::InternalCalibration* CML::parseInternalEurocCalibration(std::string path, Vector2i outputSize) {
#if CML_HAVE_YAML_CPP
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

    auto res = makeOptimalK_crop(pinhole, undistorter, Vector2i(width, height), outputSize);

    return new InternalCalibration(pinhole, Vector2(width, height), undistorter, res.first, outputSize.cast<scalar_t>());
#else
    return nullptr;
#endif

}

CML::HashMap<std::string, CML::List<std::string>> xmlDocToHashMap(rapidxml::xml_node<> *node) {

    CML::HashMap<std::string, CML::List<std::string>> result;

    for (; node; node = node->next_sibling()) {

        result[std::string(node->name(), node->name_size())].emplace_back(std::string(node->value(), node->value_size()));
    }

    return result;

}


CML::InternalCalibration* CML::parseInternalStereopolisCalibration(std::string path, Vector2i outputSize, int top, int bottom) {
    rapidxml::xml_document doc;

    std::ifstream file(path);
    std::vector<char> buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    doc.parse<0>(buffer.data());

    auto calibinternconique = xmlDocToHashMap( doc.first_node("ExportAPERO")->first_node("CalibrationInternConique")->first_node());
    auto modunif = xmlDocToHashMap( doc.first_node("ExportAPERO")->first_node("CalibrationInternConique")->first_node("CalibDistortion")->first_node("ModUnif")->first_node());

    List<scalar_t> params;
    for (const auto& paramstr : modunif["Params"]) {
        if (paramstr != "0") {
            params.emplace_back(atof(paramstr.c_str()));
        }
    }

    scalar_t f = atof(calibinternconique["F"][0].c_str());

    std::string szimgstr = calibinternconique["SzIm"][0];
    Vector2d size;
    std::sscanf(szimgstr.c_str(), "%lf %lf", &size[0], &size[1]);


    if (modunif["TypeModele"][0]=="eModele_FishEye_10_5_5") {
        PinholeUndistorter pinhole{Vector2(f, f), Vector2(params[0], params[1])};
        FishEye10_5_5 *fishEye1055 = new FishEye10_5_5({params[2], params[3], params[4], params[5], params[6]}, {params[7], params[8]}, {params[9], params[10]});


        auto res = makeOptimalK_crop(pinhole, fishEye1055, size.cast<int>(), outputSize);
        Vector4 params = res.first.getParameters();
        float nonCroppedHeight = size.y() * outputSize.x() / size.x();
        // logger.important(" NON CROPPED : " + std::to_string(nonCroppedHeight));

        params(1) *= nonCroppedHeight / outputSize.y();

        params(3) = -nonCroppedHeight/2 + std::min((bottom - top), 480)
        + 1.4f * (nonCroppedHeight - (bottom * nonCroppedHeight /size.y()));

        // logger.important("PARAMS = " + std::to_string(params(0)) + " " + std::to_string(params(1)) + " " + std::to_string(params(2)) + " " + std::to_string(params(3)));

        PinholeUndistorter newpinhole(params);

        return new InternalCalibration(pinhole, size.cast<scalar_t>(), fishEye1055, newpinhole, outputSize.cast<scalar_t>());

    }
    else {
        // todo
    }
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
