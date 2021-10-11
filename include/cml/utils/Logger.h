//
// Created by tbelos on 03/05/19.
//

#ifndef CML_LOGGER_H
#define CML_LOGGER_H

#include <string>
#include <vector>
#include <iostream>
#include <mutex>
#include <fstream>
#include <unordered_set>
#include <optional>
#include <sstream>

namespace CML {

    typedef enum {
        MORE = 0, INFO = 1, WARN = 2, IMPORTANT = 3, ERR = 4, DEADLY = 5
    } LoggerLevel;

    typedef struct {

        LoggerLevel level;
        std::string message;
        std::string formatted;
        std::string threadname;

    } LoggerMessage;

    const std::string endl = "";

    class Logger {

    public:
        class Observer {

        public:
            virtual void onNewMessage(LoggerMessage message) = 0;

        };

        inline void subscribeObserver(Observer *observer) {
            mObservers.insert(observer);
        }

        inline void removeObserver(Observer *observer) {
            mObservers.erase(observer);
        }

        Logger();

        void redirect(std::string filename);

        void setLogLevel(LoggerLevel level);

        void log(LoggerLevel level, const std::string &msg);
        void debug(const std::string &msg);
        void info(const std::string &msg);
        void warn(const std::string &msg);
        void error(const std::string &msg);
        void fatal(const std::string &msg);
        void important(const std::string &msg);

        inline Logger& operator<<(const std::string & obj) {
            if (obj == endl) {
                info(mCurrentMessage);
                mCurrentMessage = "";
            }
            else mCurrentMessage += obj;
            return *this;
        }

        inline Logger& operator<<(const char* & obj) {
            mCurrentMessage += obj;
            return *this;
        }

        inline Logger& operator<<(const unsigned char* & obj) {
            mCurrentMessage += reinterpret_cast<const char*>(obj);
            return *this;
        }

        inline Logger& operator<<(short obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(int obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(long obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(long long obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(unsigned short obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(unsigned int obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(unsigned long obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(unsigned long long obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(float obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline Logger& operator<<(double obj) {
            mCurrentMessage += std::to_string(obj);
            return *this;
        }

        inline void filter(pthread_t thread) {
            mHaveThreadFilter = true;
            mThreadFilter = thread;
        }

        inline void filterThisThread() {
            filter(pthread_self());
        }

        inline void setPrefix(std::string prefix) {
            mPrefixMutex.lock();
            mPrefix = prefix;
            mPrefixMutex.unlock();
        }

    private:
        int mStdout, mStderr;

        std::string mCurrentMessage = "";

        LoggerLevel mLevel;

        std::unordered_set<Observer*> mObservers;

        bool mHaveThreadFilter;
        pthread_t mThreadFilter;

        std::mutex mPrefixMutex;
        std::string mPrefix;

    };


    extern Logger logger;

}


#endif //CML_LOGGER_H
