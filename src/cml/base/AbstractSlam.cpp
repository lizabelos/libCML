
#include <cml/base/AbstractSlam.h>

#include <iostream>
#include <fstream>

std::string to_string(const CML::Camera& cam){
  CML::Matrix33 m = cam.getRotationMatrix();
  return "Translation:\n" +
  std::to_string(cam.getTranslation()(0)) + " " + std::to_string(cam.getTranslation()(1)) + " " + std::to_string(cam.getTranslation()(2)) +
  "\nRotation :\n" +
  std::to_string(m(0,0)) + " " + std::to_string(m(0,1)) + " " + std::to_string(m(0,2)) + "\n" +
  std::to_string(m(1,0)) + " " + std::to_string(m(1,1)) + " " + std::to_string(m(1,2)) + "\n" +
  std::to_string(m(2,0)) + " " + std::to_string(m(2,1)) + " " + std::to_string(m(2,2));
}

CML::AbstractSlam::AbstractSlam() : AbstractFunction(nullptr), mMap(), mGarbageCollectorInstance(mMap.getGarbageCollector().newInstance()) {
    //mUnusedParameters.set_empty_key("reserved.empty");
    //mUnusedParameters.set_deleted_key("reserved.deleted");

    setlocale(LC_NUMERIC, "en_US.utf8");
    mIsPaused = true;
    mIsStopped = true;
}

// Add groundtruth positions of the camera in slam
void CML::AbstractSlam::addGroundtruth(std::string pathGroundtruth){
    std::ifstream groundtruth;
    groundtruth.open(pathGroundtruth.c_str());
    if (groundtruth.is_open())
    {
        groundtruth.ignore(80,'\n');
        while(!groundtruth.eof() && groundtruth.good())
        {
            char buf[1000];
            groundtruth.getline(buf, 1000);
            int id;
            double time, r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz;
            if(13 == sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ", &time, &tx, &ty, &tz, &r11, &r12, &r13, &r21, &r22, &r23, &r31, &r32, &r33))
            {
                //camera rotation
                Matrix33 rot;
                rot << r11, r12, r13,
                       r21, r22, r23,
                       r31, r32, r33;
                //camera translation
                Vector3 translation(tx, ty, tz);
                //save camera position
                Camera cam(translation, rot);
                mGroundtruths.push_back(cam);
                //save time
                mTimes.push_back(time);
            }

        }
        groundtruth.close();
    } else {
        throw std::runtime_error("groundtruth path not found");
    }

    mHaveGroundtruth = true;

    // add correction in translation and rotation
    // correct_translation = getGroundtruth(99).getTranslation();
    // Matrix33 Rx {
    //   {1, 0, 0},
    //   {0, std::cos(M_PI/2), -std::sin(M_PI/2)},
    //   {0, std::sin(M_PI/2), std::cos(M_PI/2)}
    // };
    // correct_rotation = Rx;
    // correct_cam = getGroundtruth(104).to(Camera::identity());
    correct_cam = getGroundtruth(104);
    logger.important(to_string(correct_cam));
}

void CML::AbstractSlam::start(Ptr<AbstractCapture, NonNullable> capture) {
    interrupt();

    mCapture = capture;
    mIsStopped = false;
    mIsPaused = false;
    mNeedToRestart = true;
    mPausedNextFrame = 0;

    capture->play();
    mThread = std::thread([this](){
        while (mNeedToRestart) {
            onReset();
            mMap.reset();
            mNeedToRestart = false;
            getTimer().start();
            run();
        }
        logger.info("End of CML Thread");
    });
    pthread_setname_np(mThread.native_handle(), getName().c_str());
}

void CML::AbstractSlam::startSingleThread(Ptr<AbstractCapture, NonNullable> capture) {
    interrupt();

    mCapture = capture;
    mIsStopped = false;
    mNeedToRestart = true;
    mIsPaused = false;
    mPausedNextFrame = 0;

    mCapture->play();
    while (mNeedToRestart) {
        onReset();
        mMap.reset();
        mNeedToRestart = false;
        getTimer().start();
        run();
    }

    logger.info("SLAM have finished");
}

void CML::AbstractSlam::interrupt() {
    if (!mIsStopped) {
        stop("Interrupted");
        if (mThread.joinable()) {
            mThread.join();
        }
    }
}

void CML::AbstractSlam::wait() {
    if (!mIsStopped) {
        if (mThread.joinable()) {
            mThread.join();
        }
    }
}

void CML::AbstractSlam::setPaused(bool state) {
    mIsPaused = state;
}

bool CML::AbstractSlam::isPaused() {
    return mIsPaused;
}

void CML::AbstractSlam::stop(std::string reason) {
    logger.info("Stopping because " + reason);
    mIsStopped = true;
    if (!mCapture.isNull()) {
        mCapture->stop();
    }
}

void CML::AbstractSlam::restart() {
    logger.info("Restarting...");
    mNeedToRestart = true;
}

void CML::AbstractSlam::restartOrStop(std::string reason) {
    if (getMap().getFramesNumber() < 60) {
        return restart();
    } else {
        return stop(reason);
    }
};

void CML::AbstractSlam::next() {
    if (mIsPaused) {
        mPausedNextFrame++;
    }
}

bool CML::AbstractSlam::isStopped() {
    return mIsStopped || mNeedToRestart;
}

CML::Map &CML::AbstractSlam::getMap() {
    return mMap;
}

void CML::AbstractSlam::pauseHere() {
    setPaused(true);
    while (mIsPaused) {
        CML::usleep(1);
    }
}

CML::Ptr<CML::AbstractCapture, CML::Nullable> CML::AbstractSlam::getCapture() {
    return mCapture;
}

CML::Ptr<CML::Frame, 1> CML::AbstractSlam::getNextFrame() {

    mMap.getGarbageCollector().collect(mGarbageCollectorInstance);

    usleep(10);

    // todo : use condition variable
    getNextFrameBegin:
    if (mIsPaused && mPausedNextFrame > 0) {
        mPausedNextFrame--;
    } else if (mIsPaused) {
        CML::usleep(1);
        goto getNextFrameBegin;
    } else {
        mPausedNextFrame = 0;
    }

    if (mMemoryLimit > 0 && memoryUsage() > mMemoryLimit) {
        CML::logger.error("Memory usage exceed " + std::to_string(mMemoryLimit) + " MB.");
        stop("Memory usage");
        return {};
    }

    logger.debug("Waiting for next frame");
    Timer timer;
    timer.start();
    Ptr<CaptureImage, Nullable> captureFrame = mCapture->next();
    if (captureFrame.isNull()) {
        CML::logger.info("End of video stream");
        stop("End of video stream");
        return {};
    }
    timer.stop();
//    logger.important("Retrivied the next frame in " + std::to_string(timer.getValue()));


    mLastCaptureImage = captureFrame;

    PFrame currentFrame = mMap.createFrame(captureFrame);
    std::string resolutionsStr = "";
    for (int i = 0; i < currentFrame->getCaptureFrame().getPyramidLevels(); i++) {
        if (i != 0) resolutionsStr += " ";
        resolutionsStr += std::to_string(currentFrame->getWidth(i)) + "x" + std::to_string(currentFrame->getHeight(i));
    }

    CML::logger.raw("New frame. Id = " + std::to_string(currentFrame->getId()) + ". Pyramid resolutions : " + resolutionsStr + "\n");

    return currentFrame;
}


std::string ss;
int i = 0;
void CML::AbstractSlam::addFrame(PFrame currentFrame){
    i++;
    if (mMap.getFramesNumber() > 0) {
      //if slam have gps camera positions, initialize the new frame with gps data
      if(mHaveGroundtruth && getCapture()->imageNumbers() > 115){
          scalar_t timeFrame = getCapture()->getTime(); // time of the current frame
          int index = getCapture()->imageNumbers();
          logger.important(" index : " + std::to_string(index));
          if(timeFrame != mTimes[index]){
            logger.error("Time not synchronized with gps");
            throw std::runtime_error("error in gps data. Time not syncronized\n");
          }else{
            //initialize cam frame position
            Camera cam = getGroundtruth(index);

            Camera newCam = cam.compose(correct_cam.inverse());
            // Camera newCam = correct_cam.to(cam);
            Vector3 t = newCam.getTranslation();
            //scale ??

            Camera c(t, newCam.getRotationMatrix());
            Vector3 vec = c.getTranslation();
            // ss = ss + std::to_string(vec(0)) + " " + std::to_string(vec(1)) + " " + std::to_string(vec(2)) + "\n";
            // if(i == 150) {
            //   logger.important(ss);
            // }
            // logger.important(to_string(newCam));
            // currentFrame->setCamera(newCam);
            // logger.important(to_string(c));
            currentFrame->setCamera(c);
            // currentFrame->setCamera(newCam);
          }
      }else{
              Vector3 t = mMap.getLastFrame()->getCamera().getTranslation();
              ss = ss + std::to_string(t(0)) + " " + std::to_string(t(1)) + " " + std::to_string(t(2)) + "\n";
              if(i == 150) {
                logger.important(ss);
              }
              logger.important(to_string(mMap.getLastFrame()->getCamera()));
              currentFrame->setCamera(mMap.getLastFrame()->getCamera());
      }
      currentFrame->setExposureParameters(mMap.getLastFrame()->getExposure());
    }
    mMap.addFrame(currentFrame);

    std::string strFrame = "frame " + std::to_string(currentFrame->getId());
    std::string strPercentage;
    if (getCapture()->remaining() > 0) {
        strPercentage = std::to_string(currentFrame->getId() * 100 / (currentFrame->getId() + getCapture()->remaining() + 1)) + "%";
    } else {
        strPercentage = "realtime";
    }
    std::string strRam = "ram : " + std::to_string(memoryUsage()) + "mb";
    std::string strFps = "fps : " + std::to_string((int)getTimer().fps(currentFrame->getId()));

    logger.setPrefix(strFrame + "; " + strPercentage);
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::AbstractSlam::getLastCaptureFrame() {
    return mLastCaptureImage;
}
