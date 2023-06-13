#include "cml/utils/GarbageCollector.h"

CML::GarbageCollectorInstance CML::GarbageCollector::newInstance() {
    LockGuard lg(mNewInstanceMutex);
    if (mNumInstances == 1) {
        mThread = std::thread([&] {
            run();
        });
    }
    int id = mNumInstances++;
    mRequestedCollect.resize(mNumInstances);
    mRequestedCollect[id].start();
    return GarbageCollectorInstance(id);
}

void CML::GarbageCollector::collect(GarbageCollectorInstance instance) {
    assertThrow(instance.id >= 0, "Invalid instance");

    mRequestedCollect[instance.id].stop();

    if (mRequestedCollect[instance.id].getValue() < 0.5 || (mQueueElement.type == 0 && mQueueElement.id == instance.id)) {
        return;
    }

    if (mNumInstances > 1) {

        LockGuard lg(mMutex);

        mRequestedCollect[instance.id].start();

        mQueueElement.type = 0;
        mQueueElement.id = instance.id;
        *mQueue.getPushElement() = mQueueElement;
        mQueue.notifyPush();
    }
}

void CML::GarbageCollector::erase(Garbage *garbage) {
    if (mNumInstances <= 1) {
        delete garbage;
        return;
    }
    GarbageCollectorQueueElement queueElement;
    queueElement.type = 1;
    queueElement.ptr = garbage;

    //List<bool> list;
    //list.resize(mNumInstances, false);
    LockGuard lg(mMutex);
    *mQueue.getPushElement() = queueElement;
    mQueue.notifyPush();
    //mGarbages[garbage] = list;
}

void CML::GarbageCollector::run() {

    while (mContinue) {

        GarbageCollectorQueueElement queueElement = *mQueue.getPopElement();
        mQueue.notifyPop();

        if (!mContinue) {
            break;
        }

        if (queueElement.type == 0) {

            List<Garbage *> toRemove;

            for (auto &[garbage, list] : mGarbages) {

                list[queueElement.id] = true;
                bool canRemove = true;
                for (size_t i = 0; i < list.size(); i++) {
                    if (list[i] == false) {
                        canRemove = false;
                        break;
                    }
                }

                if (canRemove) {
                    toRemove.emplace_back(garbage);
                }

            }

            for (auto garbage : toRemove) {
                mGarbages.erase(garbage);
                delete garbage;
            }

        }

        if (queueElement.type == 1) {
            List<bool> list;
            list.resize(mNumInstances, false);
            mGarbages[queueElement.ptr] = list;
        }

    }

    for (auto [garbage, l] : mGarbages) {
        delete garbage;
    }

}