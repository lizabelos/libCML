//
// Created by belosth on 29/01/2020.
//

#ifndef CML_ALL_DRAWBOARDSHADERS_H
#define CML_ALL_DRAWBOARDSHADERS_H

#include "cml/config.h"
#include <QOpenGLShader>
#include <QOpenGLExtraFunctions>

namespace CML {

    namespace QtShaders {

        class AbstractShader {

        public:
            virtual QString vertexShaderPath() = 0;

            virtual QString fragmentShaderPath() = 0;

            virtual QStringList attributes() = 0;

            virtual QStringList uniforms() = 0;

            QString loadShaderFile(QString path) {
                path = "resources/shaders/" + path + ".glsl";
                return QString::fromStdString(readWholeBinaryFile(path.toStdString(), true));
            }

            void load(QOpenGLExtraFunctions *functions) {
                if (mIsLoaded) {
                    return;
                }

                mFunctions = functions;

                QString version = loadShaderFile("version");
                QString vertexShader = loadShaderFile(vertexShaderPath());
                QString fragmentShader = loadShaderFile(fragmentShaderPath());

                mProgram.addShaderFromSourceCode(QOpenGLShader::Vertex, version + vertexShader);
                mProgram.addShaderFromSourceCode(QOpenGLShader::Fragment, version + fragmentShader);
                mProgram.link();

                if (!mProgram.isLinked()) {
                    throw std::runtime_error("Shader not linked : " + mProgram.log().toStdString());
                }

                for (auto attribute : attributes()) {
                    mAttributes[attribute] = mProgram.attributeLocation(attribute);
                }

                for (auto uniform : uniforms()) {
                    mUniforms[uniform] = mProgram.uniformLocation(uniform);
                }

                reportError();
                mIsLoaded = true;
            }

            void bind() {
                mProgram.bind();
                reportError();
            }

            void enableAttributeArray(QString name) {
                if (mAttributes.find(name) == mAttributes.end()) {
                    throw std::runtime_error("Attribute " + name.toStdString() + " not found");
                }
                mProgram.enableAttributeArray(mAttributes[name]);
                reportError();
            }

            void disableAttributeArray(QString name) {
                if (mAttributes.find(name) == mAttributes.end()) {
                    throw std::runtime_error("Attribute " + name.toStdString() + " not found");
                }
                mProgram.disableAttributeArray(mAttributes[name]);
                reportError();
            }

            void setAttributeArray(QString name, const GLfloat *values, int size, int stride = 0) {
                if (mAttributes.find(name) == mAttributes.end()) {
                    throw std::runtime_error("Attribute " + name.toStdString() + " not found");
                }
                mProgram.setAttributeArray(mAttributes[name], values, size, stride);
                reportError();
            }

            void setUniformValue(QString name, const Eigen::Matrix4f &matrix) {
                if (mUniforms.find(name) == mUniforms.end()) {
                    throw std::runtime_error("Uniform " + name.toStdString() + " not found");
                }
                mProgram.setUniformValue(mUniforms[name], QMatrix4x4(matrix.data()).transposed());
                reportError();
            }

            void setUniformValue(QString name, GLfloat value) {
                if (mUniforms.find(name) == mUniforms.end()) {
                    throw std::runtime_error("Uniform " + name.toStdString() + " not found");
                }
                mProgram.setUniformValue(mUniforms[name], value);
                reportError();
            }

            void setUniformValue(QString name, GLfloat a, GLfloat b, GLfloat c, GLfloat d) {
                if (mUniforms.find(name) == mUniforms.end()) {
                    throw std::runtime_error("Uniform " + name.toStdString() + " not found");
                }
                mProgram.setUniformValue(mUniforms[name], QVector4D(a, b, c, d));
                reportError();
            }

            void glVertexAttribPointer(QString name, int size, GLenum type, bool normalized, int stride, const void *pointer) {
                mFunctions->glVertexAttribPointer(mAttributes[name], size, type, normalized, stride, pointer);
                reportError();
            }

            void glVertexAttribIPointer(QString name, int size, GLenum type, int stride, const void *pointer) {
                mFunctions->glVertexAttribIPointer(mAttributes[name], size, type, stride, pointer);
                reportError();
            }

            void glUniform1i(QString name, GLuint value) {
                if (mUniforms.find(name) == mUniforms.end()) {
                    throw std::runtime_error("Uniform " + name.toStdString() + " not found");
                }
                mFunctions->glUniform1i(mUniforms[name], value);
                reportError();
            }

            void glDrawArrays(GLenum mode, GLint first, GLsizei count) {
                mFunctions->glDrawArrays(mode, first, count);
                reportError();
            }

            std::string getGLErrorString(GLenum error) {
                switch (error) {
                    case GL_NO_ERROR:
                        return "GL_NO_ERROR";
                    case GL_INVALID_ENUM:
                        return "GL_INVALID_ENUM";
                    case GL_INVALID_VALUE:
                        return "GL_INVALID_VALUE";
                    case GL_INVALID_OPERATION:
                        return "GL_INVALID_OPERATION";
                    case GL_OUT_OF_MEMORY:
                        return "GL_OUT_OF_MEMORY";
                        // Add more cases for other possible error codes

                    default:
                        return "Unknown OpenGL error";
                }
            }

            void reportError() {
                bool haveError = false;
                while (GLenum err = mFunctions->glGetError() != GL_NO_ERROR) {
                    CML_LOG_FATAL("OpenGL error : " + getGLErrorString(err));
                    haveError = true;
                }
                if (haveError) {
                    throw std::runtime_error("OpenGL error");
                }
            }

        private:
            bool mIsLoaded = false;
            QOpenGLShaderProgram mProgram;
            QOpenGLExtraFunctions *mFunctions = nullptr;
            QMap<QString, int> mAttributes;
            QMap<QString, int> mUniforms;
        };

    #define registerShader(name, argVertexShader, argFragmentShader, argAttributes, argUniforms) \
        class name : public AbstractShader {               \
                                                                                           \
            public:                                                                        \
                QString vertexShaderPath() override {                                      \
                    return argVertexShader;                                                \
                }                                                                          \
                                                                                           \
                QString fragmentShaderPath() override {                                    \
                    return argFragmentShader;                                              \
                }                                                                          \
                                                                                           \
                QStringList attributes() override {                                        \
                    return QStringList() << argAttributes;                                                  \
                }                                                                          \
                                                                                           \
                QStringList uniforms() override {                                          \
                    return QStringList() << argUniforms;                                                    \
                }                                                                          \
        };

        registerShader(Color2d, "vs2d", "fscolor", "aVertexPosition", "uColor" << "uPointSize");
        registerShader(Color3d, "vs3d", "fscolor", "aVertexPosition", "uMVPMatrix" << "uColor" << "uPointSize");
        registerShader(PointCloud, "vspc", "fspc", "aColor" << "aVertexPosition" << "aVariance" << "aGroups", "uMVPMatrix" << "uVarianceFilter" << "uGroupsFilter");
        registerShader(SegmentPath, "vspath", "fspath", "aVertexPosition", "uMVPMatrix");
        registerShader(Texture, "vs2d", "fstexture", "aVertexPosition" << "aVertexTexCoord", "uTexture");

    }

}


#endif //CML_ALL_DRAWBOARDSHADERS_H
