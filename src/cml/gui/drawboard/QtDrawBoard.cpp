//
// Created by tbelos on 21/08/19.
//

#include "cml/gui/drawboard/QtDrawBoard.h"

#include <QImage>
#include <QOpenGLTexture>
#include "cml/utils/Logger.h"



inline QMatrix4x4 EigenToQt(Eigen::Matrix4f m) {
    return QMatrix4x4(m.data()).transposed();
}

CML::QtDrawBoard::QtDrawBoard(QOpenGLExtraFunctions *functions) : mFunctions(functions) {

    mColor = Eigen::Vector3f(0,0,0);
    mPointSize = 1;
    mLineWidth = 1;

    mColor2dShader.load(mFunctions);
    mColor3dShader.load(mFunctions);
    mTextureShader.load(mFunctions);
    mPointCloudShader.load(mFunctions);
    mSegmentPathShader.load(mFunctions);

}

void CML::QtDrawBoard::color(Eigen::Vector3f color) {
    mColor = color;
}

void CML::QtDrawBoard::color(float r, float g, float b) {
    mColor = Eigen::Vector3f(r, g, b);
}

void CML::QtDrawBoard::pointSize(int size) {
    mPointSize = size;
    //glPointSize(size);
}

void CML::QtDrawBoard::lineWidth(int size) {
    mLineWidth = size;
}

void CML::QtDrawBoard::disableDepthTest() {
    mFunctions->glDisable(GL_DEPTH_TEST);
}

void CML::QtDrawBoard::enableDepthTest() {
    mFunctions->glEnable(GL_DEPTH_TEST);
}

void CML::QtDrawBoard::segment(Eigen::Vector2f a, Eigen::Vector2f b) {

    a = transform2D(a);
    b = transform2D(b);

    load2DProgram();
    //glLineWidth(mLineWidth);

    GLfloat segment[] = {
            a.x(), a.y(), 1,
            b.x(), b.y(), 1
    };

    mColor2dShader.enableAttributeArray("aVertexPosition");
    mColor2dShader.setAttributeArray("aVertexPosition", (const GLfloat*)segment, 3, 0);
    mColor2dShader.glDrawArrays(GL_LINES,0,2);
    mColor2dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::point(Eigen::Vector2f a) {

    a = transform2D(a);

    load2DProgram();

    GLfloat point[] = {
            a.x(), a.y(), 1
    };

    mColor2dShader.enableAttributeArray("aVertexPosition");
    mColor2dShader.setAttributeArray("aVertexPosition", (const GLfloat*)point, 3, 0);
    mColor2dShader.glDrawArrays(GL_POINTS,0,1);
    mColor2dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::segments(std::vector<Eigen::Vector2f> points) {
    load2DProgram();
    //glLineWidth(mLineWidth);

    std::vector<Eigen::Vector3f> hpoints;
    for (auto point : points) hpoints.emplace_back(transform2D(point).homogeneous());

    mColor2dShader.enableAttributeArray("aVertexPosition");
    mColor2dShader.setAttributeArray("aVertexPosition", (const GLfloat*)hpoints.data(), 3, 0);
    mColor2dShader.glDrawArrays(GL_LINES,0,hpoints.size());
    mColor2dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::points(std::vector<Eigen::Vector2f> points) {

    load2DProgram();

    std::vector<Eigen::Vector3f> hpoints;
    for (auto point : points) hpoints.emplace_back(transform2D(point).homogeneous());


    mColor2dShader.enableAttributeArray("aVertexPosition");
    mColor2dShader.setAttributeArray("aVertexPosition", (const GLfloat*)hpoints.data(), 3, 0);
    mColor2dShader.glDrawArrays(GL_POINTS,0,hpoints.size());
    mColor2dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::segment(Eigen::Vector3f a, Eigen::Vector3f b) {
    load3DProgram();
    //glLineWidth(mLineWidth);

    GLfloat segment[] = {
            a.x(), a.y(), a.z(),
            b.x(), b.y(), b.z()
    };

    mColor3dShader.enableAttributeArray("aVertexPosition");
    mColor3dShader.setAttributeArray("aVertexPosition", (const GLfloat*)segment, 3, 0);
    mColor3dShader.glDrawArrays(GL_LINES,0,2);
    mColor3dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::point(Eigen::Vector3f a) {
    load3DProgram();

    mColor3dShader.enableAttributeArray("aVertexPosition");
    mColor3dShader.setAttributeArray("aVertexPosition", (const GLfloat*)a.data(), 3, 0);
    mColor3dShader.glDrawArrays(GL_POINTS,0,1);
    mColor3dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::segments(std::vector<Eigen::Vector3f> points) {
    load3DProgram();
    //glLineWidth(mLineWidth);

    mColor3dShader.enableAttributeArray("aVertexPosition");
    mColor3dShader.setAttributeArray("aVertexPosition", (const GLfloat*)points.data(), 3, 0);
    mColor3dShader.glDrawArrays(GL_LINES,0,points.size());
    mColor3dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::points(std::vector<Eigen::Vector3f> points) {
    load3DProgram();

    mColor3dShader.enableAttributeArray("aVertexPosition");
    mColor3dShader.setAttributeArray("aVertexPosition", (const GLfloat*)points.data(), 3, 0);
    mColor3dShader.glDrawArrays(GL_POINTS,0,points.size());
    mColor3dShader.disableAttributeArray("aVertexPosition");
}

void CML::QtDrawBoard::load2DProgram() {
    mColor2dShader.bind();
    mColor2dShader.setUniformValue("uColor", mColor.x(), mColor.y(), mColor.z(), 1);
    mColor2dShader.setUniformValue("uPointSize", mPointSize);
}

void CML::QtDrawBoard::load3DProgram() {
    mColor3dShader.bind();
    mColor3dShader.setUniformValue("uColor", mColor.x(), mColor.y(), mColor.z(), 1);
    mColor3dShader.setUniformValue("uMVPMatrix", mProjectionMatrix * mCameraMatrix * mModelMatrix);
    mColor3dShader.setUniformValue("uPointSize", mPointSize);
}

void CML::QtDrawBoard::setProjectionMatrix(Eigen::Matrix4f matrix) {
    mProjectionMatrix = matrix;
}

void CML::QtDrawBoard::setModelMatrix(Eigen::Matrix4f matrix) {
    mModelMatrix = matrix;
}

void CML::QtDrawBoard::setCameraMatrix(Eigen::Matrix4f matrix) {
    mCameraMatrix = matrix;
}

void CML::QtDrawBoard::texture(const Image &oslamImage, float x, float y, float width, float height) {
    QImage image((uchar*)oslamImage.data(), oslamImage.getWidth(), oslamImage.getHeight(), QImage::Format_RGB32);
    image = image.rgbSwapped();
    QOpenGLTexture texture(image);

    mTextureShader.bind();

    // glActiveTexture(GL_TEXTURE0);
    texture.bind();
    mTextureShader.glUniform1i("uTexture", 0);

    std::vector<Eigen::Vector3f> vertexs;
    vertexs.emplace_back(Eigen::Vector3f( x, y, 0));
    vertexs.emplace_back(Eigen::Vector3f( x + width, y, 0));
    vertexs.emplace_back(Eigen::Vector3f( x + width, y + height, 0));
    vertexs.emplace_back(Eigen::Vector3f( x, y + height, 0));

    std::vector<Eigen::Vector3f> texcoords;
    texcoords.emplace_back(Eigen::Vector3f( 0, 1, 0));
    texcoords.emplace_back(Eigen::Vector3f( 1, 1, 0));
    texcoords.emplace_back(Eigen::Vector3f( 1, 0, 0));
    texcoords.emplace_back(Eigen::Vector3f( 0, 0, 0));


    mTextureShader.enableAttributeArray("aVertexPosition");
    mTextureShader.setAttributeArray("aVertexPosition", (const GLfloat*)vertexs.data(), 3, 0);

    mTextureShader.enableAttributeArray("aVertexTexCoord");
    mTextureShader.setAttributeArray("aVertexTexCoord", (const GLfloat*)texcoords.data(), 3, 0);

    mTextureShader.glDrawArrays( GL_TRIANGLE_FAN,0,4);

    mTextureShader.disableAttributeArray("aVertexTexCoord");
    mTextureShader.disableAttributeArray("aVertexPosition");

}

void CML::QtDrawBoard::texture(PFrame &frame, float x, float y, float width, float height) {

    if (mLastCachedFrame != frame) {
        if (mLastCachedFrame.isNotNull()) {
            uncacheTexture(mLastCachedFrame);
        }
        cacheTexture(frame);
        mLastCachedFrame = frame;
    }

    mTextureShader.bind();

    // glActiveTexture(GL_TEXTURE0);
    mCachedTexture->bind();
    mTextureShader.setUniformValue("uTexture", 0);




    std::vector<Eigen::Vector3f> vertexs;
    vertexs.emplace_back(Eigen::Vector3f( x, y, 0));
    vertexs.emplace_back(Eigen::Vector3f( x + width, y, 0));
    vertexs.emplace_back(Eigen::Vector3f( x + width, y + height, 0));
    vertexs.emplace_back(Eigen::Vector3f( x, y + height, 0));

    std::vector<Eigen::Vector3f> texcoords;
    texcoords.emplace_back(Eigen::Vector3f( 0, 1, 0));
    texcoords.emplace_back(Eigen::Vector3f( 1, 1, 0));
    texcoords.emplace_back(Eigen::Vector3f( 1, 0, 0));
    texcoords.emplace_back(Eigen::Vector3f( 0, 0, 0));


    mTextureShader.enableAttributeArray("aVertexPosition");
    mTextureShader.setAttributeArray("aVertexPosition", (const GLfloat*)vertexs.data(), 3, 0);

    mTextureShader.enableAttributeArray("aVertexTexCoord");
    mTextureShader.setAttributeArray("aVertexTexCoord", (const GLfloat*)texcoords.data(), 3, 0);

    mTextureShader.glDrawArrays(GL_TRIANGLE_FAN,0,4);

    mTextureShader.disableAttributeArray("aVertexTexCoord");
    mTextureShader.disableAttributeArray("aVertexPosition");

}

void CML::QtDrawBoard::set2DArea(Eigen::Vector2f min, Eigen::Vector2f max) {
    mMinArea = min;
    mMaxArea = max;
}

void CML::QtDrawBoard::set2DAxis(Eigen::Vector2f min, Eigen::Vector2f max) {
    mMinAxis = min;
    mMaxAxis = max;
}

Eigen::Vector2f CML::QtDrawBoard::transform2D(Eigen::Vector2f input) {

    input -= mMinAxis;
    input.x() /= mMaxAxis.x() - mMinAxis.x();
    input.y() /= mMaxAxis.y() - mMinAxis.y();

    input.y() = 1-input.y();

    input.x() *= mMaxArea.x() - mMinArea.x();
    input.y() *= mMaxArea.y() - mMinArea.y();

    input += mMinArea;

    return input;

}

void CML::QtDrawBoard::finish() {

}

void CML::QtDrawBoard::pointCloud(scalar_t *coords, scalar_t *colors, unsigned int *groups, unsigned int groupsFilter, scalar_t *variance, scalar_t varianceFilter, size_t size) {

    //glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    mPointCloudShader.bind();
    mPointCloudShader.setUniformValue("uMVPMatrix", mProjectionMatrix * mCameraMatrix * mModelMatrix);
    mPointCloudShader.setUniformValue("uVarianceFilter", (GLfloat)varianceFilter);
    mPointCloudShader.glUniform1i( "uGroupsFilter", groupsFilter);

    mPointCloudShader.enableAttributeArray("aVertexPosition");
    mPointCloudShader.glVertexAttribPointer( "aVertexPosition", 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)coords);
    //mPointCloudShader.setAttributeArray("aVertexPosition", (const GLfloat *)coords, 3, 0);

    mPointCloudShader.enableAttributeArray("aColor");
    mPointCloudShader.glVertexAttribPointer( "aColor", 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)colors);
    // mPointCloudShader.setAttributeArray("aColor", (const GLfloat*)colors, 3, 0);

    mPointCloudShader.enableAttributeArray("aGroups");
    mPointCloudShader.glVertexAttribIPointer( "aGroups", 1, GL_INT, 0, (const GLint *)groups);

    mPointCloudShader.enableAttributeArray("aVariance");
    mPointCloudShader.glVertexAttribPointer( "aVariance", 1, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)variance);
    //mPointCloudShader.setAttributeArray("aVariance", (const GLfloat*)variance, 1, 0);


    mPointCloudShader.glDrawArrays(GL_POINTS,0,size);

    mPointCloudShader.disableAttributeArray("aVertexPosition");
    mPointCloudShader.disableAttributeArray("aColor");
    mPointCloudShader.disableAttributeArray("aVariance");
}

void CML::QtDrawBoard::cacheTexture(PFrame frame) {

    QImage image;
    if (frame->getCaptureFrame().haveColorImage()) {
        image = QImage((uchar *) frame->getCaptureFrame().getColorImage(0, false).data(), frame->getWidth(0),frame->getHeight(0), QImage::Format_RGB32);
        image = image.rgbSwapped();
        if (mCachedTexture == nullptr) {
            mCachedTexture = new QOpenGLTexture(image);
        } else {
            mCachedTexture->setData(image);
        }
    } else {
        //GrayImage grayImage = frame->getCaptureFrame().getGrayImage(0).cast<unsigned char>();
        //image = QImage((uchar *)grayImage.data(), grayImage.getWidth(),grayImage.getHeight(), QImage::Format_Grayscale8);
        Image colorImage = frame->getCaptureFrame().getGrayImage(0, false).cast<ColorRGBA>();
        image = QImage((uchar *) colorImage.data(), frame->getWidth(0),frame->getHeight(0), QImage::Format_RGB32);

        if (mCachedTexture == nullptr) {
            mCachedTexture = new QOpenGLTexture(image);
        } else {
            mCachedTexture->setData(image);
        }
    }

}

void CML::QtDrawBoard::uncacheTexture(PFrame frame) {


}

void CML::QtDrawBoard::cameraPath(scalar_t *coords, size_t size) {

    if (size == 0) {
        return;
    }

    //glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    mSegmentPathShader.bind();
    mSegmentPathShader.setUniformValue("uMVPMatrix", mProjectionMatrix * mCameraMatrix * mModelMatrix);

    mSegmentPathShader.enableAttributeArray("aVertexPosition");
    mSegmentPathShader.glVertexAttribPointer( "aVertexPosition", 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)coords);


    mFunctions->glDrawArrays(GL_LINE_STRIP,0,size);

    mSegmentPathShader.disableAttributeArray("aVertexPosition");

}
