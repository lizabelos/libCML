#include "cml/utils/Watchpoint.h"

#include <thread>

namespace CML {

    class WatchpointContext {

    public:
        WatchpointContext() {
            mThread = std::thread(&WatchpointContext::run, this);
        }

        ~WatchpointContext() {
            mContinue = false;
            mThread.join();
        }

        void watch(void *addr) {
            LockGuard lg(toWatchMutex);
            toWatch[addr] = *((size_t*)addr);
        }

        void unwatch(void *addr) {
            LockGuard lg(toWatchMutex);
            toWatch.erase(addr);
        }

        void run() {
            while (mContinue) {
                LockGuard lg(toWatchMutex);
                for (auto [addr, value] : toWatch) {
                    if (*((size_t*)addr) != value) {
                        abort();
                    }
                }
            }
        }

    private:
        bool mContinue = true;
        std::thread mThread;

        Mutex toWatchMutex;
        HashMap<void*, size_t> toWatch;

    };

    WatchpointContext *gWatchpointContext = nullptr;

}

CML::Watchpoint::Watchpoint(void *addr) {
    mAddr = addr;
    if (gWatchpointContext == nullptr) {
        gWatchpointContext = new WatchpointContext();
    }
    gWatchpointContext->watch(addr);
}

CML::Watchpoint::~Watchpoint() {
    gWatchpointContext->unwatch(mAddr);
}

