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

CML::QtDrawBoard::QtDrawBoard(QOpenGLExtraFunctions *functions) : QtDrawBoardShaders(), mFunctions(functions) {


    mColor = Eigen::Vector3f(0,0,0);
    mPointSize = 1;
    mLineWidth = 1;

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
    glDisable(GL_DEPTH_TEST);
}

void CML::QtDrawBoard::enableDepthTest() {
    glEnable(GL_DEPTH_TEST);
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

    m2DColorProgram.enableAttributeArray(m2DPositionLocation);
    m2DColorProgram.setAttributeArray(m2DPositionLocation, (const GLfloat*)segment, 3, 0);
    glDrawArrays(GL_LINES,0,2);
    m2DColorProgram.disableAttributeArray(m2DPositionLocation);
}

void CML::QtDrawBoard::point(Eigen::Vector2f a) {

    a = transform2D(a);

    load2DProgram();

    GLfloat point[] = {
            a.x(), a.y(), 1
    };

    m2DColorProgram.enableAttributeArray(m2DPositionLocation);
    m2DColorProgram.setAttributeArray(m2DPositionLocation, (const GLfloat*)point, 3, 0);
    glDrawArrays(GL_POINTS,0,1);
    m2DColorProgram.disableAttributeArray(m2DPositionLocation);
}

void CML::QtDrawBoard::segments(std::vector<Eigen::Vector2f> points) {
    load2DProgram();
    //glLineWidth(mLineWidth);

    std::vector<Eigen::Vector3f> hpoints;
    for (auto point : points) hpoints.emplace_back(transform2D(point).homogeneous());

    m2DColorProgram.enableAttributeArray(m2DPositionLocation);
    m2DColorProgram.setAttributeArray(m2DPositionLocation, (const GLfloat*)hpoints.data(), 3, 0);
    glDrawArrays(GL_LINES,0,hpoints.size());
    m2DColorProgram.disableAttributeArray(m2DPositionLocation);
}

void CML::QtDrawBoard::points(std::vector<Eigen::Vector2f> points) {

    load2DProgram();

    std::vector<Eigen::Vector3f> hpoints;
    for (auto point : points) hpoints.emplace_back(transform2D(point).homogeneous());


    m2DColorProgram.enableAttributeArray(m2DPositionLocation);
    m2DColorProgram.setAttributeArray(m2DPositionLocation, (const GLfloat*)hpoints.data(), 3, 0);
    glDrawArrays(GL_POINTS,0,hpoints.size());
    m2DColorProgram.disableAttributeArray(m2DPositionLocation);
}

void CML::QtDrawBoard::segment(Eigen::Vector3f a, Eigen::Vector3f b) {
    load3DProgram();
    //glLineWidth(mLineWidth);

    GLfloat segment[] = {
            a.x(), a.y(), a.z(),
            b.x(), b.y(), b.z()
    };

    m3DColorProgram.enableAttributeArray(m3DPositionLocation);
    m3DColorProgram.setAttributeArray(m3DPositionLocation, (const GLfloat*)segment, 3, 0);
    glDrawArrays(GL_LINES,0,2);
    m3DColorProgram.disableAttributeArray(m3DPositionLocation);
}

void CML::QtDrawBoard::point(Eigen::Vector3f a) {
    load3DProgram();

    m3DColorProgram.enableAttributeArray(m3DPositionLocation);
    m3DColorProgram.setAttributeArray(m3DPositionLocation, (const GLfloat*)a.data(), 3, 0);
    glDrawArrays(GL_POINTS,0,1);
    m3DColorProgram.disableAttributeArray(m3DPositionLocation);
}

void CML::QtDrawBoard::segments(std::vector<Eigen::Vector3f> points) {
    load3DProgram();
    //glLineWidth(mLineWidth);

    m3DColorProgram.enableAttributeArray(m3DPositionLocation);
    m3DColorProgram.setAttributeArray(m3DPositionLocation, (const GLfloat*)points.data(), 3, 0);
    glDrawArrays(GL_LINES,0,points.size());
    m3DColorProgram.disableAttributeArray(m3DPositionLocation);
}

void CML::QtDrawBoard::points(std::vector<Eigen::Vector3f> points) {
    load3DProgram();

    m3DColorProgram.enableAttributeArray(m3DPositionLocation);
    m3DColorProgram.setAttributeArray(m3DPositionLocation, (const GLfloat*)points.data(), 3, 0);
    glDrawArrays(GL_POINTS,0,points.size());
    m3DColorProgram.disableAttributeArray(m3DPositionLocation);
}

void CML::QtDrawBoard::load2DProgram() {

    m2DColorProgram.bind();
    m2DColorProgram.setUniformValue(m2DColorLocation, QVector4D(mColor.x(), mColor.y(), mColor.z(), 1));
    m2DColorProgram.setUniformValue(m2DPointSizeLocation, (GLfloat)mPointSize);

}

void CML::QtDrawBoard::load3DProgram() {

    m3DColorProgram.bind();
    m3DColorProgram.setUniformValue(m3DColorLocation, QVector4D(mColor.x(), mColor.y(), mColor.z(), 1));
    m3DColorProgram.setUniformValue(m3DColorMVPLocation, EigenToQt(mProjectionMatrix * mCameraMatrix * mModelMatrix));
    m3DColorProgram.setUniformValue(m3DPointSizeLocation, (GLfloat)mPointSize);

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

    m2DTextureProgram.bind();

    // glActiveTexture(GL_TEXTURE0);
    texture.bind();
    m2DTextureProgram.enableAttributeArray(m2DTextureLocation);
    m2DTextureProgram.setUniformValue(m2DTextureLocation, 0);

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


    m2DTextureProgram.enableAttributeArray(m2DTexturePositionLocation);
    m2DTextureProgram.setAttributeArray(m2DTexturePositionLocation, (const GLfloat*)vertexs.data(), 3, 0);

    m2DTextureProgram.enableAttributeArray(m2DTextureCoordsLocation);
    m2DTextureProgram.setAttributeArray(m2DTextureCoordsLocation, (const GLfloat*)texcoords.data(), 3, 0);

    glDrawArrays(GL_TRIANGLE_FAN,0,4);

    m2DTextureProgram.disableAttributeArray(m2DTextureCoordsLocation);
    m2DTextureProgram.disableAttributeArray(m2DTexturePositionLocation);
    m2DTextureProgram.disableAttributeArray(m2DTextureLocation);

}

void CML::QtDrawBoard::texture(PFrame &frame, float x, float y, float width, float height) {

    if (mLastCachedFrame != frame) {
        if (mLastCachedFrame.isNotNull()) {
            uncacheTexture(mLastCachedFrame);
        }
        cacheTexture(frame);
        mLastCachedFrame = frame;
    }

    m2DTextureProgram.bind();

    // glActiveTexture(GL_TEXTURE0);
    mCachedTexture->bind();
    m2DTextureProgram.enableAttributeArray(m2DTextureLocation);
    m2DTextureProgram.setUniformValue(m2DTextureLocation, 0);




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


    m2DTextureProgram.enableAttributeArray(m2DTexturePositionLocation);
    m2DTextureProgram.setAttributeArray(m2DTexturePositionLocation, (const GLfloat*)vertexs.data(), 3, 0);

    m2DTextureProgram.enableAttributeArray(m2DTextureCoordsLocation);
    m2DTextureProgram.setAttributeArray(m2DTextureCoordsLocation, (const GLfloat*)texcoords.data(), 3, 0);

    glDrawArrays(GL_TRIANGLE_FAN,0,4);

    m2DTextureProgram.disableAttributeArray(m2DTextureCoordsLocation);
    m2DTextureProgram.disableAttributeArray(m2DTexturePositionLocation);
    m2DTextureProgram.disableAttributeArray(m2DTextureLocation);

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

    mPCProgram.bind();
    mPCProgram.setUniformValue(mPCMVPLocation, EigenToQt(mProjectionMatrix * mCameraMatrix * mModelMatrix));
    mPCProgram.setUniformValue(mPCVarianceFilterLocation, (GLfloat)varianceFilter);
    mFunctions->glUniform1ui(mPCGroupsFilterLocation, groupsFilter);

    mPCProgram.enableAttributeArray(mPCPositionLocation);
    mFunctions->glVertexAttribPointer(mPCPositionLocation, 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)coords);
    //mPCProgram.setAttributeArray(mPCPositionLocation, (const GLfloat *)coords, 3, 0);

    mPCProgram.enableAttributeArray(mPCColorLocation);
    mFunctions->glVertexAttribPointer(mPCColorLocation, 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)colors);
    // mPCProgram.setAttributeArray(mPCColorLocation, (const GLfloat*)colors, 3, 0);

    mPCProgram.enableAttributeArray(mPCGroupsLocation);
    mFunctions->glVertexAttribIPointer(mPCGroupsLocation, 1, GL_INT, 0, (const GLint *)groups);

    mPCProgram.enableAttributeArray(mPCVarianceLocation);
    mFunctions->glVertexAttribPointer(mPCVarianceLocation, 1, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)variance);
    //mPCProgram.setAttributeArray(mPCVarianceLocation, (const GLfloat*)variance, 1, 0);


    glDrawArrays(GL_POINTS,0,size);

    mPCProgram.disableAttributeArray(mPCPositionLocation);
    mPCProgram.disableAttributeArray(mPCColorLocation);
    mPCProgram.disableAttributeArray(mPCVarianceLocation);
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

    mPathProgram.bind();
    mPathProgram.setUniformValue(mPathMVPLocation, EigenToQt(mProjectionMatrix * mCameraMatrix * mModelMatrix));

    mPathProgram.enableAttributeArray(mPathPositionLocation);
    mFunctions->glVertexAttribPointer(mPathPositionLocation, 3, CML_GL_SCALAR, GL_FALSE, 0, (const scalar_t *)coords);


    glDrawArrays(GL_LINE_STRIP,0,size);

    mPathProgram.disableAttributeArray(mPathPositionLocation);

}
