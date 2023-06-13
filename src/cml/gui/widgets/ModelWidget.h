//
// Created by tbelos on 21/08/19.
//

#ifndef CML_MODELVIEWERWIDGET_H
#define CML_MODELVIEWERWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimerEvent>
#include <QBasicTimer>
#include <QOpenGLDebugLogger>
#include <QEvent>
#include <QGestureEvent>
#include <QAction>

#include "cml/base/AbstractSlam.h"
#include "cml/map/Camera.h"
#include "cml/gui/drawboard/QtDrawBoard.h"


namespace CML {

    typedef enum {
        TRACKBALL, TOP, FIRSTPERSON, FOLLOW
    } ModelWidgetType;

    class ModelWidget : public QOpenGLWidget, protected QOpenGLExtraFunctions, public CML::Map::Observer {

        friend class MainSlamWidget;

    Q_OBJECT

    public:
        explicit ModelWidget(AbstractSlam *slam, ModelWidgetType type, QWidget *parent = nullptr);
        ~ModelWidget();

        void setVarianceFilter(scalar_t v) {
            mVarianceFilter = v;
        }

        inline void saveImagesTo(std::string path) {
            mSaveImages = true;
            mSavePath = path;
            mSaveId = 0;
        }

    protected:
        void initializeGL() override;
        void resizeGL(int w, int h) override;
        void paintGL() override;

        void mousePressEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;
        void keyPressEvent(QKeyEvent *event) override;
        void keyReleaseEvent(QKeyEvent *event) override;
        void wheelEvent(QWheelEvent *event) override;
        bool event(QEvent *event) override;

        void timerEvent(QTimerEvent *e) override;

        void paintAxis();
        void paintCameras();
        void paintMapPoints();

        Eigen::Matrix4f getTrackballMatrix() const;
        Eigen::Matrix4f getTopCameraMatrix() const;
        Eigen::Matrix4f getFirstPersonCameraMatrix() const;
        Eigen::Matrix4f getFollowMatrix() const;

        void moveFront(float delta);
        void rotateLeft(float delta);
        void rotateUp(float delta);

        void onAddFrame(CML::Map &map, CML::PFrame frame) override;

    public slots:
        void updateGroupsFilter(unsigned int groups);

    private slots:
        void onMessageLogged(const QOpenGLDebugMessage &debugMessage);

    protected slots:
        void saveImage();

    private:
        ModelWidgetType mType;

        QBasicTimer mUpdateTimer;
        QBasicTimer mGroundtruthTimer;

        AbstractSlam *mSLAM;
        float mGroundtruthScale = 0.001f;
        QtDrawBoard *mDrawBoard;

        // Trackball camera variable
        float mFront = 20.0f;
        float mLeft = 0.0f;
        float mUp = 0.0f;
        Eigen::Matrix4f mProjectionMatrix;
        Vector3 mCenter = Vector3(0,0,0), mLookat = Vector3(0,0,0), mLookUp = Vector3(0,1,0);
        float mSpeed = 1;

        Vector3 mFirstPersonCenter = Vector3(0,0,-1), mFirstPersonDirection = Vector3(0,0,1);

        QPoint mLastMousePosition;

        QOpenGLDebugLogger *mLogger;

        scalar_t mVarianceFilter = 0.000001f;

        unsigned int mGroupsFilter = 0;

        bool mKeyPressed[255];
        bool mGrabbed = false;

        GarbageCollectorInstance mGCInstance;

        bool mSaveImages = false;
        std::string mSavePath;
        int mSaveId = 0;
        CML::Queue<bool, 1> mSaveQueue;

    };

}


#endif //CML_MODELVIEWERWIDGET_H
