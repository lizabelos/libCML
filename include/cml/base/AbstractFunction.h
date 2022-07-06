//
// Created by tbelos on 18/06/19.
//

#ifndef CML_ABSTRACTFUNCTION_H
#define CML_ABSTRACTFUNCTION_H

#include <string>
#include <memory>
#include <vector>
#include <numeric>

#include <cml/config.h>
#include <cml/utils/Timer.h>
#include <cml/gui/viewer/DrawBoard.h>
#include <cml/base/Statistic.h>
#include <cml/base/Parameter.h>


namespace CML {



    class AbstractFunction : public DeterministicallyHashable {

    public:
        AbstractFunction(Ptr<AbstractFunction, Nullable> parent) : mParent(parent) {
            //mChildFunctions.set_empty_key((AbstractFunction*)1);
            if (parent.isNotNull()) {
                parent->mChildFunctions.insert(this);
            }
        }

        virtual ~AbstractFunction() {
            if (mParent.isNotNull()) {
                mParent->mChildFunctions.erase(this);
            }
        }

        virtual std::string getName() {
            return "Abstract Function";
        }

        inline std::string getAlias() {
            return mAlias;
        }

        void setAlias(const std::string &s) {
            mAlias = s;
            refreshParameters();
        }

        virtual void viewOnCapture(DrawBoard &drawBoard, PFrame frame) {
            for (auto child : mChildFunctions) {
                if (child->isViewableOnCapture()) child->viewOnCapture(drawBoard, frame);
            }
        }

        virtual void viewOnReconstruction(DrawBoard &drawBoard) {
            for (auto child : mChildFunctions) {
                if (child->isViewableOnModel()) child->viewOnReconstruction(drawBoard);
            }
        }

        virtual bool canShowPoint(PPoint mapPoint) {
            return true;
        }

        List<Ptr<AbstractFunction, NonNullable>> getChildFunctions() {
            return List<Ptr<AbstractFunction, NonNullable>>(mChildFunctions.begin(), mChildFunctions.end());
        }

        Timer &getTimer() {
            return mTimer;
        }

        inline bool isViewableOnCapture() const {
            return mViewableOnCapture;
        }

        inline bool isViewableOnModel() const {
            return mViewableOnModel;
        }

        inline void setViewable(bool v) {
            mViewableOnCapture = v;
            mViewableOnModel = v;
        }

        inline void setViewableOnCapture(bool v) {
            mViewableOnCapture = v;
        }

        inline void setViewableOnModel(bool v) {
            mViewableOnModel = v;
        }

        virtual Map &getMap() {
            return mParent->getMap();
        }

        virtual void onNewParameter(std::string name, Parameter &parameter) {
            if (mAlias.empty()) {
                return;
            }
            mParent->onNewParameter(mAlias + "." + name, parameter);
        }

        template <typename ...Params> Parameter createParameter(std::string name, Params... params) {
            Parameter parameter(name, std::forward<Params>(params)...);
            mParameters.emplace_back(parameter);
            onNewParameter(name, parameter);
            return parameter;
        }

        PStatistic createStatistic(std::string name) {
            PStatistic statistic = new Statistic(getMap(), STATISTIC_NORMAL, name);
            mStatistics.emplace_back(statistic);
            return statistic;
        }

        PStatistic createAveragedStatistic(std::string name) {
            PStatistic statistic = new Statistic(getMap(), STATISTIC_AVERAGE, name);
            mStatistics.emplace_back(statistic.p());
            return statistic;
        }

        Ptr<StatisticTimer, NonNullable> createStatisticTimer(std::string name) {
            Ptr<StatisticTimer, NonNullable> statistic = new StatisticTimer(getMap(), name);
            mStatistics.emplace_back(statistic.p());
            return statistic;
        }

        List<PStatistic> getStatistics() {
            return mStatistics;
        }

        List<Parameter> getParameters() {
            return mParameters;
        }

        void refreshParameters() {
            for (auto parm : getParameters()) {
                logger.important("Set Parameters : " + parm.name());
                onNewParameter(parm.name(), parm);
            }
            for (auto child : getChildFunctions()) {
                if (child->getAlias() != "") {
                    logger.important("Refreshing " + child->getAlias());
                    child->refreshParameters();
                }
            }
        }

    private:
        Ptr<AbstractFunction, Nullable> mParent;
        Set<AbstractFunction*> mChildFunctions;

        List<Parameter> mParameters;
        List<PStatistic> mStatistics;

        Timer mTimer;
        bool mViewableOnCapture = false, mViewableOnModel = false;

        std::string mAlias;

    };

}


#endif //CML_ABSTRACTFUNCTION_H
