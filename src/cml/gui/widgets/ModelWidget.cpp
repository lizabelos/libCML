//
// Created by tbelos on 21/08/19.
//

#include "cml/gui/widgets/ModelWidget.h"
#include "cml/utils/Logger.h"

inline float radians(float angle) {
    return angle * M_PI / 180.0f;
}

template<class T> inline Eigen::Matrix<T, 4, 4> perspective(double fovy, double aspect, double zNear, double zFar) {

    assert(aspect > 0);
    assert(zFar > zNear);

    double radf = radians(fovy);

    double tanHalfFovy = tan(radf / 2.0);
    Eigen::Matrix<T, 4, 4> res = Eigen::Matrix<T, 4, 4>::Zero();
    res(0, 0) = 1.0 / (aspect * tanHalfFovy);
    res(1, 1) = 1.0 / (tanHalfFovy);
    res(2, 2) = -(zFar + zNear) / (zFar - zNear);
    res(3, 2) = -1.0;
    res(2, 3) = -(2.0 * zFar * zNear) / (zFar - zNear);
    return res;
}

CML::ModelWidget::ModelWidget(CML::AbstractSlam *slam, ModelWidgetType type, QWidget *parent)
: QOpenGLWidget(parent), QOpenGLExtraFunctions(), mType(type), mSLAM(slam) {
    mDrawBoard = nullptr;
    switch (mType) {
        case TRACKBALL:
            mFront = 2;
            break;
        case TOP:
            mFront = 7;
            break;
        default:
            break;
    }
    for (int i = 0; i < 255; i++) {
        mKeyPressed[i] = false;
    }
    slam->getMap().subscribeObserver(this);
}

CML::ModelWidget::~ModelWidget() {
    mSLAM->getMap().removeObserver(this);
}

void CML::ModelWidget::initializeGL() {
    initializeOpenGLFunctions();

    const GLubyte *glVersion = glGetString(GL_VERSION);
    logger.info((std::string)"GL Version : " + std::string(reinterpret_cast<const char*>(glVersion)));

    mLogger = new QOpenGLDebugLogger(this);
    mLogger->initialize();
    connect(mLogger, &QOpenGLDebugLogger::messageLogged, this, &ModelWidget::onMessageLogged);
    mLogger->startLogging();

    mDrawBoard = new QtDrawBoard(this);

    QOpenGLWidget::initializeGL();

    mUpdateTimer.start(1000 / 15, this);
    mGroundtruthTimer.start(1000, this);

    mGCInstance = mSLAM->getMap().getGarbageCollector().newInstance();

}

void CML::ModelWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    mProjectionMatrix = perspective<float>(90.0f, (float)w / (float)h, 0.001f, 1000000000.0f);

    mDrawBoard->setProjectionMatrix(mProjectionMatrix);

    QOpenGLWidget::resizeGL(w, h);
}

void CML::ModelWidget::paintGL() {

    mSLAM->getMap().getGarbageCollector().collect(mGCInstance);

    glClearColor(0, 0, 0, 1.0f);
    // glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    paintAxis();
    paintCameras();
    paintMapPoints();

    mSLAM->viewOnReconstruction(*mDrawBoard);

    mDrawBoard->finish();

}

void CML::ModelWidget::onAddFrame(CML::Map &map, CML::PFrame lastFrame) {

    Vector3 lastCenter = lastFrame->getCamera().eye();
    lastCenter.x() = -lastCenter.x();
    lastCenter = -lastCenter;

    Vector3 lastLookat = lastFrame->getCamera().look();
    lastLookat.x() = -lastLookat.x();
    lastLookat = -lastLookat;

    Vector3 lastUp = lastFrame->getCamera().up() - lastFrame->getCamera().eye();
    lastUp.normalize();
    lastUp.x() = -lastUp.x();
    lastUp = -lastUp;

    const CML::scalar_t factor = 0.9;

    mCenter = mCenter * factor + lastCenter * (1.0 - factor);
    mLookat = mLookat * factor + lastLookat * (1.0 - factor);
    mSpeed = mSpeed * factor + (map.getLastFrame(std::min(map.getFramesNumber() - 1, (unsigned int)120))->getCamera().eye() - lastFrame->getCamera().eye()).norm() * (1.0 - factor);

    if (mType == FOLLOW) {
        mLookUp = mLookUp * factor + lastUp * (1.0 - factor);


        Vector3 direction = lastLookat - lastCenter;
        Vector3 center = lastCenter - (direction * mSpeed);
        center = center + (mLookUp * mSpeed); //Eigen::Vector3f(0,mSpeed,0);


        mFirstPersonCenter = center * factor + mFirstPersonCenter * (1 - factor);
        mFirstPersonDirection = mFirstPersonDirection * factor + (mLookat - mFirstPersonCenter) * (1.0 - factor);
    }



    if (mSaveImages) {
        QMetaObject::invokeMethod(this, "saveImage", Qt::QueuedConnection);
        mSaveQueue.getPopElement();
        mSaveQueue.notifyPop();
    }

}

void CML::ModelWidget::saveImage() {
    QImage img(this->size(), QImage::Format_RGB32);
    QPainter painter(&img);
    this->render(&painter);

    std::string id = std::to_string(mSaveId);
    while (id.size() < 5) {
        id = "0" + id;
    }

    img.save(QString::fromStdString(mSavePath + "/" + id + ".jpg"), "JPG", 100);
    mSaveId++;

    *mSaveQueue.getPushElement() = true;
    mSaveQueue.notifyPush();
}


void CML::ModelWidget::paintAxis() {
    mDrawBoard->color(1, 1, 1);
    mDrawBoard->lineWidth(1);

    mDrawBoard->color(1,0,0);
    mDrawBoard->segment(Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0));
    mDrawBoard->color(0,1,0);
    mDrawBoard->segment(Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,-1,0));
    mDrawBoard->color(0,0,1);
    mDrawBoard->segment(Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1));
}

void CML::ModelWidget::paintCameras() {

    switch (mType) {

        case TRACKBALL:
            mDrawBoard->setCameraMatrix(getTrackballMatrix());
            break;
        case TOP:
            mDrawBoard->setCameraMatrix(getTopCameraMatrix());
            break;
        case FIRSTPERSON:
            mDrawBoard->setCameraMatrix(getFirstPersonCameraMatrix());
            break;
        case FOLLOW:
            mDrawBoard->setCameraMatrix(getFollowMatrix());
    }

    switch (mType) {
        case TRACKBALL:
        case TOP: {
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            Eigen::Vector3f center = mCenter.cast<float>();
            center(0) = -center(0);
            center(2) = -center(2);
            T.col(3) = center.homogeneous();

            mDrawBoard->setModelMatrix(T);
            break;
        }
        default:
            mDrawBoard->setModelMatrix(Eigen::Matrix4f::Identity());
            break;
    }


    mDrawBoard->enableDepthTest();
   // mDrawBoard->disableDepthTest();

    mDrawBoard->lineWidth(3);

    size_t s = mSLAM->getMap().getCameraCenters().size();
    for (size_t i = 0; i < s; i++) {
        mDrawBoard->cameraPath(mSLAM->getMap().getCameraCenters()[i], mSLAM->getMap().getCameraCentersSize()[i]);
    }

    mDrawBoard->lineWidth(3);
    mDrawBoard->color(0,1,0);
    Optional<Camera> lastCamera;
    for (auto camera : mSLAM->getMap().getAlignedGroudtruth()) {
        if (!lastCamera.has_value()) {
            lastCamera = camera;
            continue;
        }
        //mDrawBoard->paintCamera(camera);
        mDrawBoard->segment((Eigen::Vector3f)lastCamera.value().eye().cast<float>(), (Eigen::Vector3f)camera.eye().cast<float>());
        lastCamera = camera;
    }

    if (mSLAM->getMap().getFramesNumber() > 0) {
        mDrawBoard->lineWidth(5);
        mDrawBoard->color(1, 1, 1);
        mDrawBoard->paintCamera(mSLAM->getMap().getLastFrame()->getCamera());
    }

}

void CML::ModelWidget::paintMapPoints() {

    mDrawBoard->pointSize(1);

    size_t s = mSLAM->getMap().getBufferGroups().size();
    for (size_t i = 0; i < s; i++) {
        mDrawBoard->pointCloud(
                mSLAM->getMap().getBufferCoordinates()[i],
                mSLAM->getMap().getBufferColors()[i],
                mSLAM->getMap().getBufferGroups()[i],
                mGroupsFilter,
                mSLAM->getMap().getUncertainties()[i],
                mVarianceFilter,
                CML_MAP_MAPPOINT_BUFFER_SIZE
        );
    }

}

void CML::ModelWidget::updateGroupsFilter(unsigned int groups) {
    mGroupsFilter = groups;
}

Eigen::Matrix4f CML::ModelWidget::getTrackballMatrix() const {
    Eigen::Matrix<float, 3, 1> xRotation = Eigen::Vector3f(1.0f, 0.0f, 0.0f).transpose() * Eigen::AngleAxisf((float)(mLeft * 3.14), Eigen::Vector3f(0.0f, 1.0f, 0.0f)).matrix();


    Eigen::Affine3f viewMatrix = Eigen::Translation3f(Eigen::Vector3f(0.0f, 0.0f, -mFront)) *
                                 Eigen::AngleAxisf(mLeft * 3.14f, Eigen::Vector3f(0.0f, 1.0f, 0.0f)) *
                                 Eigen::AngleAxisf(mUp * 3.14f, xRotation.transpose());
    return viewMatrix.matrix();
}


Eigen::Matrix4f CML::ModelWidget::getTopCameraMatrix() const {
    Eigen::Affine3f viewMatrix = Eigen::Translation3f(Eigen::Vector3f(0.0f, 0.0f, -mFront)) *
                                 Eigen::AngleAxisf(3.14f / 2.0f, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    return viewMatrix.matrix();
}

/// @brief Returns a view transformation matrix like the one from glu's lookAt
/// @see http://www.opengl.org/sdk/docs/man2/xhtml/gluLookAt.xml
/// @see glm::lookAt
CML::Matrix44 lookAt(const CML::Vector3 &eye, const CML::Vector3 &center, const CML::Vector3 &up){
    CML::Vector3 f = (center - eye).normalized();
    CML::Vector3 u = up.normalized();
    CML::Vector3 s = f.cross(u).normalized();
    u = s.cross(f);
    CML::Matrix44 mat = CML::Matrix44::Zero();
    mat(0,0) = s.x();
    mat(0,1) = s.y();
    mat(0,2) = s.z();
    mat(0,3) = -s.dot(eye);
    mat(1,0) = u.x();
    mat(1,1) = u.y();
    mat(1,2) = u.z();
    mat(1,3) = -u.dot(eye);
    mat(2,0) = -f.x();
    mat(2,1) = -f.y();
    mat(2,2) = -f.z();
    mat(2,3) = f.dot(eye);
    mat.row(3) << 0,0,0,1;
    return mat;
}


Eigen::Matrix4f CML::ModelWidget::getFirstPersonCameraMatrix() const {
    return lookAt(mFirstPersonCenter, mFirstPersonCenter + mFirstPersonDirection, mLookUp).cast<float>();
}

Eigen::Matrix4f CML::ModelWidget::getFollowMatrix() const {
    /*Vector3f direction = mLookat - mCenter;
    Vector3f center = mCenter - (direction * mSpeed);
    center = center + (mLookUp * mSpeed); //Eigen::Vector3f(0,mSpeed,0);
    return lookAt(center, mLookat, mLookUp);*/
    return lookAt(mFirstPersonCenter, mFirstPersonCenter + mFirstPersonDirection, mLookUp).cast<float>();
}

void CML::ModelWidget::moveFront(float delta) {
    mFront *= delta;
}

void CML::ModelWidget::rotateLeft(float delta) {
    mLeft += delta;
    mFirstPersonDirection = Eigen::AngleAxis<scalar_t>(-delta, Vector3(0,1,0)) * mFirstPersonDirection;
}

void CML::ModelWidget::rotateUp(float delta) {
    mUp += delta;
    mFirstPersonDirection = Eigen::AngleAxis<scalar_t>(delta, Vector3(1,0,0)) * mFirstPersonDirection;
}

void CML::ModelWidget::mousePressEvent(QMouseEvent *event) {
    mLastMousePosition = cursor().pos();
    if (event->buttons() & Qt::LeftButton) {
        mGrabbed = !mGrabbed;
        if (mGrabbed) {
            if (mType == FOLLOW) {
                mType = FIRSTPERSON;
            }
            grabKeyboard();
            grabMouse();
            setFocus();
            setCursor(Qt::BlankCursor);
            setMouseTracking(true);
        } else {
            if (mType == FIRSTPERSON) {
                mType = FOLLOW;
            }
            releaseKeyboard();
            releaseMouse();
            setCursor(Qt::ArrowCursor);
            setMouseTracking(false);
        }
    }
}

void CML::ModelWidget::mouseMoveEvent(QMouseEvent *event) {
    if (mGrabbed) {
        int dx = cursor().pos().x() - mLastMousePosition.x();
        int dy = cursor().pos().y() - mLastMousePosition.y();
        rotateLeft((float)dx * 0.01f);
        rotateUp((float)dy * 0.01f);
        cursor().setPos(mapToGlobal(QPoint(width() / 2, height() / 2)));
        mLastMousePosition = cursor().pos();
    }

}

void CML::ModelWidget::keyPressEvent(QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_Z:
            mKeyPressed[0] = true;
            break;
        case Qt::Key_Q:
            mKeyPressed[1] = true;
            break;
        case Qt::Key_S:
            mKeyPressed[2] = true;
            break;
        case Qt::Key_D:
            mKeyPressed[3] = true;
            break;
        case Qt::Key_Space:
            mKeyPressed[4] = true;
            break;
        case Qt::Key_Shift:
            mKeyPressed[5] = true;
            break;
        default:
            break;
    }
}

void CML::ModelWidget::keyReleaseEvent(QKeyEvent *event) {
    switch (event->key()) {
        case Qt::Key_Z:
            mKeyPressed[0] = false;
            break;
        case Qt::Key_Q:
            mKeyPressed[1] = false;
            break;
        case Qt::Key_S:
            mKeyPressed[2] = false;
            break;
        case Qt::Key_D:
            mKeyPressed[3] = false;
            break;
        case Qt::Key_Space:
            mKeyPressed[4] = false;
            break;
        case Qt::Key_Shift:
            mKeyPressed[5] = false;
            break;
        default:
            break;
    }
}

void CML::ModelWidget::wheelEvent(QWheelEvent *event) {

    int y = event->angleDelta().y();
    if (y > 0) {
        moveFront(1.1f);
    }
    if (y < 0) {
        moveFront(1.0 / 1.1f);
    }

}

void CML::ModelWidget::timerEvent(QTimerEvent *e) {

    if (e->timerId() == mUpdateTimer.timerId()) {

        if (mType == FIRSTPERSON && mGrabbed) {
            float ratio = 0.03;
            if (mKeyPressed[0] == true) { // forward
                mFirstPersonCenter += mFirstPersonDirection * ratio;
            }
            if (mKeyPressed[1] == true) { // Left
                mFirstPersonCenter -= mFirstPersonDirection.cross(Vector3(0,1,0)) * ratio;
            }
            if (mKeyPressed[2] == true) { // backward
                mFirstPersonCenter -= mFirstPersonDirection * ratio;
            }
            if (mKeyPressed[3] == true) { // right
                mFirstPersonCenter += mFirstPersonDirection.cross(Vector3(0,1,0)) * ratio;
            }
            if (mKeyPressed[4] == true) { // up
                mFirstPersonCenter.y() += ratio;
            }
            if (mKeyPressed[5] == true) { // down
                mFirstPersonCenter.y() -= ratio;
            }
        }

        update();
        return;
    }

    if (e->timerId() == mGroundtruthTimer.timerId()) {
        mSLAM->getMap().refreshErrorFromGroundtruth();
    }

}

void CML::ModelWidget::onMessageLogged(const QOpenGLDebugMessage &debugMessage) {

    switch (debugMessage.type()) {

        case QOpenGLDebugMessage::InvalidType:
            logger.error(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::ErrorType:
            logger.error(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::DeprecatedBehaviorType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::UndefinedBehaviorType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PortabilityType:
            logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::PerformanceType:
            //logger.warn(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::OtherType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::MarkerType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPushType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::GroupPopType:
            //logger.info(debugMessage.message().toStdString());
            break;
        case QOpenGLDebugMessage::AnyType:
            //logger.info(debugMessage.message().toStdString());
            break;
    }

}

bool CML::ModelWidget::event(QEvent *event) {
    if (event->type() == QEvent::Gesture) {
        if (QGesture *pinch = static_cast<QGestureEvent*>(event)->gesture(Qt::PinchGesture)) {
            moveFront(static_cast<QPinchGesture*>(pinch)->scaleFactor());
        }
    }
    return QOpenGLWidget::event(event);
}

