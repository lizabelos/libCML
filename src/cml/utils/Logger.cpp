//
// Created by tbelos on 03/05/19.
//

#include "cml/utils/Logger.h"
#include <fcntl.h>

#if ANDROID
#include <QDebug>
#endif

namespace CML {
    CML::Logger logger;
}

CML::Logger::Logger() : mLevel(MORE) {

}

void CML::Logger::log(LoggerLevel level, const std::string &msg) {

    if (level < mLevel) {
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

#if ANDROID
    qDebug() << QString::fromStdString(loggerMessage.formatted);
#else
    std::cout << loggerMessage.formatted;
#endif

    for (auto observer : mObservers) {
        observer->onNewMessage(loggerMessage);
    }

}

void CML::Logger::raw(const std::string &msg) {
#if ANDROID
    qDebug() << QString::fromStdString(msg);
#else
    std::cout << msg;
#endif
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
