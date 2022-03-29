//
// Created by tbelos on 03/05/19.
//

#include "cml/utils/Logger.h"
#include <unistd.h>
#include <fcntl.h>

namespace CML {
    CML::Logger logger;
}

CML::Logger::Logger() : mLevel(MORE) {

    mStdout = dup(STDOUT_FILENO);
    mStderr = dup(STDOUT_FILENO);
/*
#ifdef WIN32
    int devNull = open("nul", O_WRONLY);
    dup2(devNull, STDOUT_FILENO);
    dup2(devNull, STDERR_FILENO);
#else
    int devNull = open("/dev/null", O_WRONLY);
    dup2(devNull, STDOUT_FILENO);
    dup2(devNull, STDERR_FILENO);
#endif*/
}

void CML::Logger::redirect(std::string filename) {
    mStdout = open(filename.c_str(), O_WRONLY);
    mStderr = mStdout;
}

void CML::Logger::log(LoggerLevel level, const std::string &msg) {

    if (level < mLevel) {
        return;
    }

    if (mHaveThreadFilter && pthread_self() != mThreadFilter) {
        return;
    }

    //char threadName[1024];
    //pthread_getname_np(pthread_self(), threadName, 1024);

    LoggerMessage loggerMessage;
    loggerMessage.level = level;
    loggerMessage.message = msg;
    //loggerMessage.threadname = threadName;

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string datetimeStr(buffer);

    std::string levelStr;
    switch (level) {

        case MORE: levelStr = "DEBUG"; break;
        case INFO: levelStr = "INFO"; break;
        case WARN: levelStr = "WARN"; break;
        case ERR: levelStr = "ERROR"; break;
        case DEADLY: levelStr = "DEADLY"; break;
        case IMPORTANT: levelStr = "IMPORTANT"; break;
    }

    mPrefixMutex.lock();
    loggerMessage.formatted = loggerMessage.threadname + "(" + mPrefix + ") : [" + levelStr + "] " + msg + "\n";
    mPrefixMutex.unlock();

    //if (level >= mLevel && (!mHaveThreadFilter || pthread_self() == mThreadFilter)) {

        if (level == MORE || level == INFO || level == IMPORTANT) {
            write(mStdout, loggerMessage.formatted.c_str(), loggerMessage.formatted.size());
        }
        else {
            write(mStderr, loggerMessage.formatted.c_str(), loggerMessage.formatted.size());
        }

    //}

    for (auto observer : mObservers) {
        observer->onNewMessage(loggerMessage);
    }

}

void CML::Logger::raw(const std::string &msg) {
    write(mStdout, msg.c_str(), msg.size());
}

void CML::Logger::debug(const std::string &msg) {
    log(LoggerLevel::MORE, msg);
}

void CML::Logger::info(const std::string &msg) {
    log(LoggerLevel::INFO, msg);
}

void CML::Logger::warn(const std::string &msg) {
    log(LoggerLevel::WARN, msg);
}

void CML::Logger::error(const std::string &msg) {
    log(LoggerLevel::ERR, msg);
}

void CML::Logger::fatal(const std::string &msg) {
    log(LoggerLevel::DEADLY, msg);
}

void CML::Logger::setLogLevel(CML::LoggerLevel level) {
    mLevel = level;
}

void CML::Logger::important(const std::string &msg) {
    log(LoggerLevel::IMPORTANT, msg);
}
