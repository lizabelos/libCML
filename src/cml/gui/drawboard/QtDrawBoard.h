//
// Created by tbelos on 21/08/19.
//

#ifndef CML_DRAWBOARDFORQT_H
#define CML_DRAWBOARDFORQT_H

#include "cml/gui/viewer/DrawBoard.h"
#include "cml/image/Array2D.h"
#include "cml/map/Map.h"
#include "QtDrawBoardShaders.h"

#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>

namespace CML {

    class QtDrawBoard : public DrawBoard {

    public:
        QtDrawBoard(QOpenGLExtraFunctions *functions);

        void setProjectionMatrix(Eigen::Matrix4f matrix);
        void setModelMatrix(Eigen::Matrix4f matrix);
        void setCameraMatrix(Eigen::Matrix4f matrix);

        void color(Eigen::Vector3f color) override;
        void color(float r, float g, float b) override;

        void pointSize(int size) override;
        void lineWidth(int size) override;

        void disableDepthTest() override;
        void enableDepthTest() override;

        void segment(Eigen::Vector2f a, Eigen::Vector2f b) override;
        void point(Eigen::Vector2f a) override;

        void segments(std::vector<Eigen::Vector2f> points) override;
        void points(std::vector<Eigen::Vector2f> points) override;

        void segment(Eigen::Vector3f a, Eigen::Vector3f b) override;
        void point(Eigen::Vector3f a) override;

        void segments(std::vector<Eigen::Vector3f> points) override;
        void points(std::vector<Eigen::Vector3f> points) override;

        void texture(const Image &image, float x, float y, float width, float height);
        void texture(PFrame &frame, float x, float y, float width, float height);


        void set2DArea(Eigen::Vector2f min, Eigen::Vector2f max);
        void set2DAxis(Eigen::Vector2f min, Eigen::Vector2f max);

        Eigen::Vector2f transform2D(Eigen::Vector2f input);

        void pointCloud(scalar_t *coords, scalar_t *colors, unsigned int *groups, unsigned int groupsFilter, scalar_t *variance, scalar_t varianceFilter, size_t size);
        void cameraPath(scalar_t *coords, size_t size);


        void finish() override;

        void cacheTexture(PFrame frame);
        void uncacheTexture(PFrame frame);


    protected:
        void load2DProgram();
        void load3DProgram();

    private:
        QOpenGLExtraFunctions *mFunctions;

        Eigen::Vector3f mColor;
        int mLineWidth, mPointSize;

        Eigen::Matrix4f mProjectionMatrix = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f mModelMatrix = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f mCameraMatrix = Eigen::Matrix4f::Identity();

        Eigen::Vector2f mMinArea = Eigen::Vector2f(-1, -1);
        Eigen::Vector2f mMaxArea = Eigen::Vector2f(1, 1);
        Eigen::Vector2f mMinAxis = Eigen::Vector2f(-1, -1);
        Eigen::Vector2f mMaxAxis = Eigen::Vector2f(1, 1);

        OptPFrame mLastCachedFrame;
        QOpenGLTexture *mCachedTexture = nullptr;

        QtShaders::Color2d mColor2dShader;
        QtShaders::Color3d mColor3dShader;
        QtShaders::PointCloud mPointCloudShader;
        QtShaders::SegmentPath mSegmentPathShader;
        QtShaders::Texture mTextureShader;

    };

}


#endif //CML_DRAWBOARDFORQT_H
