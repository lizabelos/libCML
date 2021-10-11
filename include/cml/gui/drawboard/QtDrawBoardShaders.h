//
// Created by belosth on 29/01/2020.
//

#ifndef CML_ALL_DRAWBOARDSHADERS_H
#define CML_ALL_DRAWBOARDSHADERS_H

#include <QOpenGLShader>

namespace CML {

    class QtDrawBoardShaders {

    public:
        QtDrawBoardShaders();

    protected:
        QOpenGLShaderProgram m2DColorProgram, m3DColorProgram, m2DTextureProgram, mPCProgram, mPathProgram;
        int m3DColorMVPLocation, m2DColorLocation, m3DColorLocation, m2DTextureLocation, mPCColorLocation;
        int m3DPositionLocation, m2DPositionLocation, m2DTexturePositionLocation, m2DTextureCoordsLocation, mPCPositionLocation, mPathPositionLocation;
        int m2DPointSizeLocation, m3DPointSizeLocation, mPCVarianceLocation, mPCVarianceFilterLocation, mPCGroupsLocation, mPCGroupsFilterLocation;
        int mPCMVPLocation, mPathMVPLocation;

    };

}


#endif //CML_ALL_DRAWBOARDSHADERS_H
