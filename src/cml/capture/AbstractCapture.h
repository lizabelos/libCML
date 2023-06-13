//
// Created by tbelos on 16/05/19.
//

#ifndef CML_ABSTRACTCAPTURE_H
#define CML_ABSTRACTCAPTURE_H

#include "cml/config.h"
#include "CaptureImage.h"
#include "cml/utils/Timer.h"
#include <thread>

namespace CML {

    class AbstractCapture {

    public:
        virtual ~AbstractCapture() = default;

        virtual void play() = 0;

        virtual void stop() = 0;

        virtual Ptr<CaptureImage, Nullable> next() = 0;

        virtual int remaining() {
            return 0;
        }

        virtual int imageNumbers() {
            return 0;
        }

        virtual inline bool isRealtime() {
            return false;
        }

        inline void revere() {
            mReverse = !mReverse;
        }

        inline bool isReverse() {
            return mReverse;
        }

        virtual inline Ptr<CaptureImageGenerator, Nullable> getGenerator() {
            return nullptr;
        }

    private:
        bool mReverse = false;

    };

    class AbstractFiniteCapture : public AbstractCapture {

    public:
        virtual ~AbstractFiniteCapture() = default;

    };

    class AbstractRealtimeCapture : public AbstractCapture {

    public:
        virtual ~AbstractRealtimeCapture() = default;

        virtual void setExposure(float exposure) = 0;

        virtual float getMinimumExposure() = 0;

        virtual float getMaximumExposure() = 0;

        virtual void setAutoExposure(bool value) = 0;

        virtual bool isAutoExposure() = 0;

        inline bool isRealtime() final {
            return true;
        }

    };

    class AbstractMultithreadFiniteCapture : public AbstractCapture {

    public:
        AbstractMultithreadFiniteCapture() {

        }

        void play() final {
            if (!mThreadContinue) {
                mThreadContinue = true;
                mThread = std::thread(&AbstractMultithreadFiniteCapture::run, this);
                mTimer.start();
                mCurrentImageCount = 0;
            }
        }

        void stop() final {
            mThreadContinue = false;
        }

        Ptr<CaptureImage, Nullable> next() final {
            Ptr<CaptureImage, Nullable> result = *mQueue.getPopElement();
            if (result.isNull()) {
                mThreadContinue = false;
                mQueue.notifyPop();
                return result;
            }
            if (mFirstTime < 0) {
                mFirstTime = result->getTime();
            }
            mQueue.notifyPop();
            mCurrentImageCount++;
            // mTimer.waitFor(result->getTime() - mFirstTime);
            return result;
        }

    protected:
        virtual Ptr<CaptureImage, Nullable> multithreadNext() = 0;

        void run() {
            while (mThreadContinue) {
                *mQueue.getPushElement() = multithreadNext();
                mQueue.notifyPush();
            }
        }

        bool isStopped() const {
            return !mThreadContinue;
        }

    private:
        Queue<Ptr<CaptureImage, Nullable>, 1> mQueue;
        std::thread mThread;
        bool mThreadContinue = false;
        Timer mTimer;
        int mCurrentImageCount;
        scalar_t mFirstTime = -1;
    };

}


#endif //CML_ABSTRACTCAPTURE_H
