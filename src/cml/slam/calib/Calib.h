#include <cml/slam/modslam/Hybrid.h>

using namespace CML;

class Calib : public Hybrid {

    using CornerAndDescriptor = Features::ORB;
    using Descriptor = CornerAndDescriptor::Descriptor;


public:
    Calib() : Hybrid() {
        /// WARNING : Calib absolutely needs 640x480 resolution

        mDistortionMapX = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMapY = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMapWeight = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMap = Array2D<Vector2f>(640 / mDistortionDiviser, 480 / mDistortionDiviser, Vector2f(0, 0));
        mTotalDistortionMap = Array2D<Vector2f>(640, 480, Vector2f(0, 0));

        for (int y = 0; y < 480; y++) {
            for (int x = 0; x < 640; x++) {
                mTotalDistortionMap(x, y) = Vector2f(x, y);
            }
        }
    }

    void saveDistortionMap(const std::string& filename) {
        FILE *f = fopen(filename.c_str(), "wb");
        if (f == nullptr) {
            std::cerr << "Could not open file " << filename << std::endl;
            return;
        }
        fwrite((const void *)mDistortionMap.eigenMatrix().data(), sizeof(Vector2f), mDistortionMap.eigenMatrix().size(), f);
        fclose(f);
    }

    void saveDistortionMapAsBmp(const std::string &filename) {
        Array2D<ColorRGBA> image(640, 480);
        float maxDistortion = 0;
        for (int y = 0; y < 480; y++) {
            for (int x = 0; x < 640; x++) {
                float distortion = mDistortionMap(x, y).norm();
                maxDistortion = std::max(maxDistortion, distortion);
            }
        }
        for (int y = 0; y < 480; y++) {
            for (int x = 0; x < 640; x++) {
                float distortion = mDistortionMap(x, y).norm() * 255.0 / maxDistortion;
                image(x, y) = ColorRGBA(distortion);
            }
        }
        image.saveBmp(filename);
    }

    void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final {

        List<DistortedVector2d> distortedCorners;
        distortedCorners.emplace_back(DistortedVector2d(0, 0));
        distortedCorners.emplace_back(DistortedVector2d(640, 0));
        distortedCorners.emplace_back(DistortedVector2d(640, 480));
        distortedCorners.emplace_back(DistortedVector2d(0, 480));

        List<DistortedVector2d> toDraw;

        for (auto corner : distortedCorners) {
            auto undistorted = mReferenceFrame->undistort(corner, 0);
            WorldPoint wp = WorldPoint::fromInverseDepth(1, undistorted, mReferenceFrame->getCamera());
            toDraw.emplace_back(frame->distort(wp.project(frame->getCamera()),0));
        }

        drawBoard.color(1,0,0);
        drawBoard.lineWidth(5);

        drawBoard.segment((Vector2f)toDraw[0].cast<float>(), (Vector2f)toDraw[1].cast<float>());
        drawBoard.segment((Vector2f)toDraw[1].cast<float>(), (Vector2f)toDraw[2].cast<float>());
        drawBoard.segment((Vector2f)toDraw[2].cast<float>(), (Vector2f)toDraw[3].cast<float>());
        drawBoard.segment((Vector2f)toDraw[3].cast<float>(), (Vector2f)toDraw[0].cast<float>());

    }

protected:

    void beforeStart() override {
        auto referenceImage = loadImage("resources/reference.jpg", true).first;
        referenceImage = referenceImage.resize(640, 480);

        CML::Vector2 originalSize(640, 480);
        CML::PinholeUndistorter undistorter(CML::Vector2(1.0, 1.7778), CML::Vector2(0.5, 0.5));
        undistorter = undistorter.scaleAndRecenter(originalSize, CML::Vector2(-0.5, -0.5));

        if (this->getCapture().isNull()) {
            abort();
        }

        while (this->getCapture()->getGenerator().isNull()) {
            CML_LOG_IMPORTANT("Waiting for capture to be set");
            CML::OS::usleep(1000000);
        }

        Ptr<CaptureImage, Nullable> captureFrame = this->getCapture()->getGenerator()->create()
                .setImage(referenceImage)
                .setTime(0)
                .setCalibration(new CML::InternalCalibration(undistorter, originalSize))
                .setExposure(1)
                .generate();
        mReferenceFrame = this->getMap().createFrame(captureFrame);
        this->getMap().addFrame(mReferenceFrame);

        setFlatFixedReferenceFrame(mReferenceFrame, mDirectFid);
    }

    void onTracked(PFrame frame) override {
        directOptimizeDistortionMap(frame);
    }

    void onNewFrame(PFrame currentFrame) override {
        currentFrame->getCaptureFrame().postRemoveDistortion(mTotalDistortionMap);
    }

    void directOptimizeDistortionMap(PFrame currentFrame) {

        mDistortionMapX = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMapY = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMapWeight = Array2D<float>(640 / mDistortionDiviser, 480 / mDistortionDiviser, 0.0f);
        mDistortionMap = Array2D<Vector2f>(640 / mDistortionDiviser, 480 / mDistortionDiviser, Vector2f(0, 0));

        int maxIterations[] = {10, 20, 50, 50, 50};
        int maxLevel = std::min(currentFrame->getCaptureFrame().getPyramidLevels() - 1, 4);

        for (int level = maxLevel; level >= 0; level--) {

            for (int iteration = 0; iteration < maxIterations[level]; iteration++) {

                for (int pointId = 0; pointId < mReferenceFrame->getFeaturePoints(mDirectFid).size(); pointId++) {

                    optimizePoint(currentFrame, mReferenceFrame->getFeaturePoints(mDirectFid)[pointId].point(level), level);

                }

                for (int y = 0; y < mDistortionMap.getHeight(); y++) {
                    for (int x = 0; x < mDistortionMap.getWidth(); x++) {
                        mDistortionMap(x,y) = Vector2f(mDistortionMapX(x,y), mDistortionMapY(x,y)) / mDistortionMapWeight(x,y);
                    }
                }

            }

        }

        //mTotalDistortionMap.eigenMatrix() += mDistortionMap.eigenMatrix();
        for (int y = 0; y < mTotalDistortionMap.getHeight(); y++) {
            for (int x = 0; x < mTotalDistortionMap.getWidth(); x++) {
                mTotalDistortionMap(x,y) += mDistortionMap.interpolate(Vector2f((float)x / (float)mDistortionDiviser, (float)y / (float)mDistortionDiviser));
            }
        }


    }

    void optimizePoint(PFrame currentFrame, DistortedVector2d referenceDistorted, int level) {

        UndistortedVector2d referenceUndistorted = currentFrame->undistort(referenceDistorted, level);
        WorldPoint point3d = WorldPoint::fromInverseDepth(1, referenceUndistorted, mReferenceFrame->getCamera());
        UndistortedVector2d currentUndistorted = point3d.project(currentFrame->getCamera());
        DistortedVector2d currentDistorted = currentFrame->distort(currentUndistorted, level);
        DistortedVector2d currentDistorted0 = currentFrame->distort(currentUndistorted, 0);
        DistortedVector2d currentDistortedDivided = DistortedVector2d(currentDistorted0 / mDistortionDiviser);

        if (currentDistortedDivided.x() < 3 || currentDistortedDivided.y() < 3 || currentDistortedDivided.x() > mDistortionMap.getWidth() - 3 || currentDistortedDivided.y() > mDistortionMap.getHeight() - 3) {
            return;
        }

        Vector2f currentDistorted2 = currentDistorted.cast<float>() + mDistortionMap.interpolate(currentDistortedDivided.cast<float>());

        if (!mReferenceFrame->isInside(referenceDistorted, level, 3)) {
            return;
        }

        if (!currentFrame->isInside(currentDistorted2.cast<scalar_t>(), level, 3)) {
            return;
        }

        Vector3f referenceColor = mReferenceFrame->getCaptureFrame().getDerivativeImage(level).interpolate(referenceDistorted.cast<float>());
        Vector3f currentColor = currentFrame->getCaptureFrame().getDerivativeImage(level).interpolate(currentDistorted2);
        Vector2f referenceGradient = referenceColor.tail<2>();

        Vector2f affLL = mReferenceFrame->getExposure().to(currentFrame->getExposure()).getParameters().cast<float>();
        scalar_t residual = currentColor[0] - (float)(affLL[0] * referenceColor[0] + affLL[1]);

        // compute lucas kanade optical flow
        // https://en.wikipedia.org/wiki/Lucas%E2%80%93Kanade_method

        Vector2f A = referenceGradient;
        float b = residual;
        // A * delta = b
        // delta = A^-1 * b
        Vector2f delta = A.inverse() * b;
        if (delta.norm() > 1) {
            delta = delta / delta.norm();
        }


        int size = 16;
        float sigma = 2;

        for (int x = -size; x <= size; x++) {
            for (int y = -size; y <= size; y++) {
                Vector2f currentDistorted2 = currentDistortedDivided.cast<float>() + Vector2f(x, y);
                if (currentDistorted2.x() < 0 || currentDistorted2.y() < 0 || currentDistorted2.x() >= mDistortionMap.getWidth() || currentDistorted2.y() >= mDistortionMap.getHeight()) {
                    continue;
                }
                float weight = exp(-((x * x + y * y) / (2 * sigma * sigma)));
                mDistortionMapX(currentDistorted2.cast<int>()) += delta.x() * weight;
                mDistortionMapY(currentDistorted2.cast<int>()) += delta.y() * weight;
                mDistortionMapWeight(currentDistorted2.cast<int>()) += weight;
            }
        }





    }


private:
    PFrame mReferenceFrame;

    Array2D<Vector2f> mTotalDistortionMap, mDistortionMap;
    Array2D<float> mDistortionMapX, mDistortionMapY, mDistortionMapWeight;

    int mDirectFid;

    const int mDistortionDiviser = 4;

};
