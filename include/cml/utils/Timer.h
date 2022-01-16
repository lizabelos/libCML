#ifndef CML_TIMER_H
#define CML_TIMER_H

#include <cml/config.h>
#include <chrono>
#include <thread>

namespace CML {

    class Timer {

    public:
        class Observer : public DeterministicallyHashable {

        public:
            virtual void onNewTimerValue(const Timer &timer, scalar_t value) = 0;

        };

        Timer() {
            //mObservers.set_empty_key((Observer*)1);
            //mObservers.set_deleted_key((Observer*)2);
            mValue = 0.0;
        }

        void start() {
            mStart = std::chrono::high_resolution_clock::now();
        }

        void waitFor(scalar_t seconds) {
            //std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<scalar_t> duration(seconds);
            std::this_thread::sleep_until(mStart + duration);
            //mValue = std::chrono::duration<scalar_t>(end - mStart).count();
            //if (mValue >= seconds) return;
            //std::this_thread::sleep_for((end - mStart) - duration);
        }

        void stop() {
            std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
            mValue = std::chrono::duration<scalar_t>(end - mStart).count();
            for (auto observer : mObservers) {
                observer->onNewTimerValue(*this, mValue);
            }
        }

        void stopAndStart() {
            std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
            mValue = std::chrono::duration<scalar_t>(end - mStart).count();
            for (auto observer : mObservers) {
                observer->onNewTimerValue(*this, mValue);
            }
            mStart = end;
        }

        void stopAndPrint(const std::string &comment) {
            stop();
            // logger.info(comment + " : " + std::to_string(getValue()));
        }

        void stopAndPrintToCout(std::string comment) {
            std::cout << comment << " : " << getValue() << std::endl;
        }

        scalar_t getValue() const {
            return mValue;
        }

        scalar_t fps(size_t num) const {
            std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
            return (scalar_t)num / (scalar_t)std::chrono::duration<scalar_t>(end - mStart).count();
        }

        inline void subscribeObserver(Observer *observer) {
            assertThrow(observer != nullptr, "null observer passed to subscribeObserver");
            mObservers.insert(observer);
        }

        inline void removeObserver(Observer *observer) {
            mObservers.erase(observer);
        }

    private:
        scalar_t mValue;
        std::chrono::time_point<std::chrono::high_resolution_clock> mStart;
        Set<Observer*> mObservers;

    };

}

#endif