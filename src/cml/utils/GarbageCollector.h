#ifndef CML_GARBAGECOLLECTOR_H
#define CML_GARBAGECOLLECTOR_H

#include <cml/config.h>
#include <cml/utils/Timer.h>
#include <thread>

namespace CML {

    class Garbage : public DeterministicallyHashable {

    public:
        virtual ~Garbage() = default;

    };

    class GarbageCollectorInstance {
        friend class GarbageCollector;

    public:
        inline GarbageCollectorInstance() {
            this->id = -1;
        }

    private:
        inline GarbageCollectorInstance(int id) {
            this->id = id;
        }

        int id;

    };

    typedef struct GarbageCollectorQueueElement {

        int type;
        Garbage *ptr;
        int id;

    } GarbageCollectorQueueElement;

    class GarbageCollector {

    public:
        inline GarbageCollector() : mQueue() {
            mNumInstances = 0;
            mRequestedCollect.reserve(1000);
        }

        inline ~GarbageCollector() {
            mContinue = false;
            mQueue.notifyPush();
            if (mNumInstances > 1) {
                mThread.join();
            }
        }

        GarbageCollectorInstance newInstance();

        void collect(GarbageCollectorInstance instance);

        void erase(Garbage *garbage);

        inline int numGarbages() {
            return mGarbages.size();
        }

    protected:
        void run();

    private:
        bool mContinue = true;
        Atomic<int> mNumInstances;
        Queue<GarbageCollectorQueueElement, 131072> mQueue;
        HashMap<Garbage*, List<bool>> mGarbages;
        Mutex mMutex, mNewInstanceMutex;
        std::thread mThread;
        GarbageCollectorQueueElement mQueueElement;
        List<Timer> mRequestedCollect;

    };


}

#endif