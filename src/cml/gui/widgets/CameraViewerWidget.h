//
// Created by tbelos on 22/08/19.
//

#ifndef CML_CAMERAVIEWERWIDGET_H
#define CML_CAMERAVIEWERWIDGET_H

#include "cml/base/AbstractSlam.h"
#include "cml/gui/drawboard/QtDrawBoard.h"

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QTimerEvent>
#include <QBasicTimer>
#include <QOpenGLDebugLogger>
#include <QAction>


namespace CML {

    class CameraViewerWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions {

    Q_OBJECT

    public:
        explicit CameraViewerWidget(AbstractSlam *slam, int keyFrame, QWidget *parent = nullptr);
        ~CameraViewerWidget();

    protected:
        void initializeGL() override;
        void resizeGL(int w, int h) override;
        void paintGL() override;

        void timerEvent(QTimerEvent *e) override;

    private slots:
        void onMessageLogged(const QOpenGLDebugMessage &debugMessage);

    private:
        QBasicTimer mTimer;

        AbstractSlam *mSLAM;
        QtDrawBoard *mDrawBoard;

        QOpenGLDebugLogger *mLogger;

        int mKeyFrame;


    };

}


#endif //CML_CAMERAVIEWERWIDGET_H
