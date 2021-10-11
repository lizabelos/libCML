#include "cml/base/Statistic.h"
#include "cml/base/AbstractFunction.h"

CML::StatisticsSheet::StatisticsSheet(Map &map, std::string path) {
    mOutputFile.open(path);
    map.subscribeObserver(this);
    mNumbers = 0;
    mCurrentFrameId = -1;
    mHaveChange = false;
}

CML::StatisticsSheet::~StatisticsSheet() {
    mOutputFile.close();
    for (auto [statistic, id] : mStatisticToId) {
        statistic->removeObserver(this);
    }
}

void CML::StatisticsSheet::registerStatistic(PStatistic statistic) {
    mStatisticToId[statistic.p()] = mNumbers;
    mIdToStatistic[mNumbers] = statistic.p();
    mNumbers++;
    statistic->subscribeObserver(this);
    mCurrentValues.resize(mNumbers, 0);
}

void CML::StatisticsSheet::registerFunction(AbstractFunction *function) {
    for (PStatistic statistic : function->getStatistics()) {
        registerStatistic(statistic);
    }
}

void CML::StatisticsSheet::onNewValue(Statistic *statistic, scalar_t x, scalar_t y) {
    // todo : take x into account
    mCurrentValues[mStatisticToId[statistic]] = y;
    mHaveChange = true;
}

void CML::StatisticsSheet::onAddFrame(Map &map, PFrame frame) {
    if (mCurrentFrameId == -1) {
        // DUMP THE TITLE TO THE FILE
        for (int i = 0; i < mNumbers; i++) {
            mOutputFile << mIdToStatistic[i]->getName() << ";";
        }
        mOutputFile << std::endl;
    } else {
        if (mHaveChange) {
            for (int i = 0; i < mNumbers; i++) {
                mOutputFile << mCurrentValues[i] << ";";
            }
            mOutputFile << std::endl;
        }
    }
    mCurrentFrameId = frame->getId();
    mHaveChange = false;
}

void CML::StatisticsSheet::onFrameChangeGroup(Map &map, PFrame frame, int groupId, bool state) {

}
