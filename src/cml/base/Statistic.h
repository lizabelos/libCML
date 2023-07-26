#ifndef CML_STATISTIC_H
#define CML_STATISTIC_H

#include <cml/config.h>
#include <cml/map/Map.h>

#include <fstream>

namespace CML {

    typedef enum StatisticMode {
        STATISTIC_NORMAL, STATISTIC_AVERAGE
    } StatisticMode;

    class Statistic : public Map::Observer {

        friend class AbstractFunction;

    public:
        class Observer : public DeterministicallyHashable {

        public:
            virtual void onNewValue(Statistic *statistic, scalar_t x, scalar_t y) = 0;

        };

        inline void subscribeObserver(Observer *observer) {
            assertThrow(observer != nullptr, "null observer passed to subscribeObserver");
            LockGuard lg(mObserversMutex);
            mObservers.insert(observer);
        }

        inline void removeObserver(Observer *observer) {
            LockGuard lg(mObserversMutex);
            mObservers.erase(observer);
        }

        inline void addValue(scalar_t y) {
            LockGuard lg(mObserversMutex);
            mWaitingValues.emplace_back(y);
        }

        inline void addValue(scalar_t x, scalar_t y) {
            LockGuard lg(mObserversMutex);

            for (Observer *observer : mObservers) {
                observer->onNewValue(this, x, y);
            }
        }

        inline std::string getName() const {
            return mName;
        }

        void onAddFrame(Map &map, PFrame frame) final {
            if (mMode == STATISTIC_NORMAL) {
                for (size_t i = 0; i < mWaitingValues.size(); i++) {
                    addValue((scalar_t)mMap.getLastFrame()->getId() + (scalar_t) i / (scalar_t) mWaitingValues.size(), mWaitingValues[i]);
                }
            } else {
                if (!mWaitingValues.empty()) {
                    scalar_t accumulator = 0;
                    for (size_t i = 0; i < mWaitingValues.size(); i++) {
                        accumulator += mWaitingValues[i];
                    }
                    accumulator /= (scalar_t)mWaitingValues.size();
                    addValue(mMap.getLastFrame()->getId(), accumulator);
                }
            }
            mWaitingValues.clear();
        }

    protected:
        Statistic(Map &map, StatisticMode mode, std::string name) : mMap(map), mMode(mode), mName(std::move(name)) {
            //mObservers.set_empty_key((Observer*)1);
            //mObservers.set_deleted_key((Observer*)2);
            mMap.subscribeObserver(this);
        }

        ~Statistic() {
            mMap.removeObserver(this);
        }

    private:
        Map &mMap;
        StatisticMode mMode;

        Set<Observer*> mObservers;
        Mutex mObserversMutex;

        std::string mName;

        List<scalar_t> mWaitingValues;

    };

    class StatisticTimer : public Statistic {
        friend class AbstractFunction;

    public:
        void start() {
            mTimer.start();
        }

        void stop() {
            mTimer.stop();
            addValue(mTimer.getValue());
        }

    protected:
        StatisticTimer(Map &map, std::string name) : Statistic(map, STATISTIC_NORMAL, std::move(name)) {
        }

        ~StatisticTimer() {
        }

    private:
        Timer mTimer;
    };

    using PStatistic = Ptr<Statistic, NonNullable>;

    class StatisticsSheet : public Statistic::Observer, public Map::Observer {

    public:
        explicit StatisticsSheet(Map &map, std::string path);

        ~StatisticsSheet();

        void registerStatistic(PStatistic statistic);

        void registerFunction(AbstractFunction *function);

        void onNewValue(Statistic *statistic, scalar_t x, scalar_t y) final;

        void onAddFrame(Map &map, PFrame frame) final;

        void onFrameChangeGroup(Map &map, PFrame frame, int groupId, bool state) final;


    private:
        std::ofstream mOutputFile;
        int mNumbers;
        int mCurrentFrameId;
        HashMap<Statistic*, int> mStatisticToId;
        HashMap<int, Statistic*> mIdToStatistic;
        List<scalar_t> mCurrentValues;
        bool mHaveChange;

    };

}

#endif