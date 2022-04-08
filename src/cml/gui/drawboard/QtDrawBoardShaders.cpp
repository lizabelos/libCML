//
// Created by belosth on 29/01/2020.
//

#include "cml/gui/drawboard/QtDrawBoardShaders.h"

#include <cassert>

#include <QOpenGLFunctions>
#include <QImage>
#include <QOpenGLTexture>
#include "cml/utils/Logger.h"

const QString glslVs2D(
        "in vec3 aVertexPosition;\n"
        "in vec3 aVertexTexCoord;\n"
        "\n"
        "out vec3 vPosition;\n"
        "out vec2 vTexCoord;\n"
        "\n"
        "uniform mat4 uMVPMatrix;\n"
        "uniform float uPointSize;\n"
        "\n"
        "void main() {\n"
        "    vPosition = aVertexPosition;\n"
        "    vTexCoord = aVertexTexCoord.xy;\n"
        "\n"
        "    gl_Position = vec4(vPosition.x, vPosition.y, 0, 1);\n"
        "    gl_PointSize = uPointSize;\n"
        "}"
);

const QString glslVs3D(
        "in vec3 aVertexPosition;\n"
        "in vec3 aVertexTexCoord;\n"
        "\n"
        "out vec2 vTexCoord;\n"
        "\n"
        "uniform mat4 uMVPMatrix;\n"
        "uniform float uPointSize;\n"
        "\n"
        "void main() {\n"
        "    vTexCoord = vec2(aVertexTexCoord.x,-aVertexTexCoord.y);\n"
        "    vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);\n"
        "    gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));\n"
        "    gl_PointSize = uPointSize;\n"
        "}\n"
        ""
);

const QString glslFsColor(
        "in vec2 vTexCoord;\n"
        "out vec4 fFragColor;\n"
        "\n"
        "uniform vec4 uColor;\n"
        "\n"
        "void main() {\n"
        "     fFragColor = uColor;\n"
        "}"
);

const QString glslFsTexture(
        "in vec2 vTexCoord;\n"
        "out vec4 fFragColor;\n"
        "\n"
        "uniform sampler2D uTexture;\n"
        "\n"
        "void main() {\n"
        "    fFragColor = texture(uTexture,vTexCoord);\n"
        "}"
);

#if ANDROID
const QString glslVsPC(
        "in vec3 aVertexPosition;\n"
        "in vec3 aColor;\n"
        "in float aVariance;\n"
        "in int aGroups;\n"
        "\n"
        "out vec3 vColor;\n"
        "out vec4 vPosition;\n"
        "\n"
        "uniform mat4 uMVPMatrix;\n"
        "uniform float uVarianceFilter;\n"
        "uniform int uGroupsFilter;\n"
        "\n"
        "void main() {\n"
        "        vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);\n"
        "        gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));\n"
        "        gl_PointSize = 3.0;\n"
        "        vColor = aColor;\n"
        "    vPosition = gl_Position;\n"
        "}\n"
);
#else
const QString glslVsPC(
        "in vec3 aVertexPosition;\n"
        "in vec3 aColor;\n"
        "in float aVariance;\n"
        "in int aGroups;\n"
        "\n"
        "out vec3 vColor;\n"
        "out vec4 vPosition;\n"
        "\n"
        "uniform mat4 uMVPMatrix;\n"
        "uniform float uVarianceFilter;\n"
        "uniform int uGroupsFilter;\n"
        "\n"
        "void main() {\n"
        "    int noGroup = 0;\n"
        "    if ((uGroupsFilter & aGroups) == noGroup || aVariance > uVarianceFilter) {\n"
        "        gl_Position = vec4(-999999, -999999, -999999, 1);\n"
        "        vColor = vec3(0, 0, 0);\n"
        "    } else {\n"
        "        vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);\n"
        "        gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));\n"
        "        gl_PointSize = 3.0;\n"
        "        vColor = aColor;\n"
        "    }\n"
        "    vPosition = gl_Position;\n"
        "}\n"
);
#endif

const QString glslFsPC(
        "out vec4 fFragColor;\n"
        "\n"
        "in vec3 vColor;\n"
        "in vec4 vPosition;\n"
        "\n"
        "void main() {\n"
        "     // float far = 10.0;\n"
        "     // float near = 1.0;\n"
        "     // float ndcDepth = (2.0 * vPosition.z - near - far) / (far - near);\n"
        "     // float clipDepth = clamp(ndcDepth * 0.5 / vPosition.w + 0.5, 0.0, 1.0);\n"
        "     // fFragColor = vec4(clipDepth, 0.0, 1.0 - clipDepth, 1.0);\n"
        "     fFragColor = vec4(vColor, 1);\n"
        "}"
);

const QString glslVsPath(
        "in vec3 aVertexPosition;\n"
        "\n"
        "out vec3 vColor;\n"
        "out vec4 vPosition;\n"
        "\n"
        "uniform mat4 uMVPMatrix;\n"
        "\n"
        "void main() {\n"
        "    vec3 invertedVertexPosition = vec3(aVertexPosition.x, -aVertexPosition.y, -aVertexPosition.z);\n"
        "    gl_Position = (uMVPMatrix * vec4(invertedVertexPosition, 1));\n"
        "    vColor = vec3(1, 1, 1);\n"
        "    vPosition = gl_Position;\n"
        "}\n"
);

const QString glslFsPath(
        "out vec4 fFragColor;\n"
        "\n"
        "in vec3 vColor;\n"
        "in vec4 vPosition;\n"
        "\n"
        "void main() {\n"
        "     fFragColor = vec4(vColor, 1);\n"
        "}"
);

CML::QtDrawBoardShaders::QtDrawBoardShaders() {

    // logger << "Loading shaders..." << endl;

    QString openglVersion;

    openglVersion = "#version 310 es\n"
                    "#undef lowp\n"
                    "#undef mediump\n"
                    "#undef highp\n"
                    "\n"
                    "precision lowp float;\n"
                    "\n"
                    "\n";




    // logger.info("Loading 2D color shader from sources...");
    m2DColorProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, openglVersion + glslVs2D);
    m2DColorProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, openglVersion + glslFsColor);
    // logger.info("Linking the 2D color shader...");
    m2DColorProgram.link();

    std::string msg = m2DColorProgram.log().toStdString();
    if (msg != "") {
        logger.error(msg);
    }

    // logger.info("Binding the 2D color shader...");
    m2DColorProgram.bind();
    m2DColorLocation = m2DColorProgram.uniformLocation("uColor");
    m2DPositionLocation = m2DColorProgram.attributeLocation("aVertexPosition");
    m2DPointSizeLocation = m2DColorProgram.uniformLocation("uPointSize");




    // logger.info("Loading the 3D color shader from soucres...");
    m3DColorProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, openglVersion + glslVs3D);
    m3DColorProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, openglVersion + glslFsColor);
    // logger.info("Linking the 3D color shader...");
    m3DColorProgram.link();

    msg = m3DColorProgram.log().toStdString();
    if (msg != "") {
        logger.error(msg);
    }

    m3DColorProgram.bind();
    // logger.info("Binding the 3D color shader...");
    m3DColorMVPLocation = m3DColorProgram.uniformLocation("uMVPMatrix");
    m3DColorLocation = m3DColorProgram.uniformLocation("uColor");
    m3DPositionLocation = m3DColorProgram.attributeLocation("aVertexPosition");
    m3DPointSizeLocation = m3DColorProgram.uniformLocation("uPointSize");



    // logger.info("Loading the PC color shader from soucres...");
    mPCProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, openglVersion + glslVsPC);
    mPCProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, openglVersion + glslFsPC);
    // logger.info("Linking the PC color shader...");
    mPCProgram.link();

    msg = mPCProgram.log().toStdString();
    if (msg != "") {
        logger.error(msg);
    }

    mPCProgram.bind();
    // logger.info("Binding the PC color shader...");
    mPCMVPLocation = mPCProgram.uniformLocation("uMVPMatrix");
    mPCColorLocation = mPCProgram.attributeLocation("aColor");
    mPCPositionLocation = mPCProgram.attributeLocation("aVertexPosition");
    mPCVarianceLocation =  mPCProgram.attributeLocation("aVariance");
    mPCVarianceFilterLocation = mPCProgram.uniformLocation("uVarianceFilter");
    mPCGroupsLocation = mPCProgram.attributeLocation("aGroups");
    mPCGroupsFilterLocation = mPCProgram.uniformLocation("uGroupsFilter");





    // logger.info("Loading the PC color shader from soucres...");
    mPathProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, openglVersion + glslVsPath);
    mPathProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, openglVersion + glslFsPath);
    // logger.info("Linking the PC color shader...");
    mPathProgram.link();

    msg = mPathProgram.log().toStdString();
    if (msg != "") {
        logger.error(msg);
    }

    mPathProgram.bind();
    // logger.info("Binding the PC color shader...");
    mPathMVPLocation = mPathProgram.uniformLocation("uMVPMatrix");
    mPathPositionLocation = mPCProgram.attributeLocation("aVertexPosition");




    // logger.info("Loading the 2D texture shader...");
    m2DTextureProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, openglVersion + glslVs2D);
    m2DTextureProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, openglVersion + glslFsTexture);
    // logger.info("Linking the 2D texture shader...");
    m2DTextureProgram.link();

    msg = m2DTextureProgram.log().toStdString();
    if (msg != "") {
        logger.error(msg);
    }

    m2DTextureProgram.bind();
    // logger.info("Binding the 3D color shader...");
    m2DTextureLocation = m2DTextureProgram.uniformLocation("uTexture");
    m2DTexturePositionLocation = m2DTextureProgram.attributeLocation("aVertexPosition");
    m2DTextureCoordsLocation = m2DTextureProgram.attributeLocation("aVertexTexCoord");

}
