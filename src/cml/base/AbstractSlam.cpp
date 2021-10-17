
#include <cml/base/AbstractSlam.h>

CML::AbstractSlam::AbstractSlam() : AbstractFunction(nullptr), mMap(), mGarbageCollectorInstance(mMap.getGarbageCollector().newInstance()) {
    //mUnusedParameters.set_empty_key("reserved.empty");
    //mUnusedParameters.set_deleted_key("reserved.deleted");

    setlocale(LC_NUMERIC, "en_US.utf8");
    mIsPaused = true;
    mIsStopped = true;
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
    logger.important("Retrivied the next frame in " + std::to_string(timer.getValue()));


    mLastCaptureImage = captureFrame;

    PFrame currentFrame = mMap.createFrame(captureFrame);
    std::string resolutionsStr = "";
    for (int i = 0; i < currentFrame->getCaptureFrame().getPyramidLevels(); i++) {
        if (i != 0) resolutionsStr += " ";
        resolutionsStr += std::to_string(currentFrame->getWidth(i)) + "x" + std::to_string(currentFrame->getHeight(i));
    }

    CML::logger.important("New frame. Id = " + std::to_string(currentFrame->getId()) + ". Pyramid resolutions : " + resolutionsStr);

    return currentFrame;
}

void CML::AbstractSlam::addFrame(PFrame currentFrame) {
    if (mMap.getFramesNumber() > 0) {
        currentFrame->setCamera(mMap.getLastFrame()->getCamera());
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
    std::string strFps = "fps : " + std::to_string(getTimer().fps(currentFrame->getId()));

    logger.setPrefix(strFrame + "; " + strPercentage + "; " + strRam + "; " + strFps);
}

CML::Ptr<CML::CaptureImage, CML::Nullable> CML::AbstractSlam::getLastCaptureFrame() {
    return mLastCaptureImage;
}

