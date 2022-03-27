//
// Created by belosth on 03/01/2020.
//

#ifndef CML_PHOTOMETRIC_H
#define CML_PHOTOMETRIC_H

#include <cml/config.h>
#include <cml/optimization/dso/DSOContext.h>
#include <cml/optimization/dso/DSOPoint.h>
#include <cml/features/cornerTracker/CornerMatcher.h>
#include <cml/optimization/Triangulation.h>
#include <cml/features/corner/PixelSelector.h>

namespace CML {

    namespace Optimization {

        class DSOTracerPointPrivate : public PrivateDataStructure {

        public:
            List<PFrame> frames;
            DSOTracerStatus lastTraceStatus = IPS_UNINITIALIZED;
            scalar_t iDepthMin = 1.0 / 1000.0;
            scalar_t iDepthMax = NAN;
            Vector2 lastTraceUV = Vector2(-1, -1);
            scalar_t lastTracePixelInterval = -1;
            Matrix22 gradH;
            List<scalar_t> weights;
            scalar_t energyTH = NAN;
            scalar_t quality = 10000;
            scalar_t lastResidual = NAN;
            float my_type = 1;
            FeatureIndex referenceIndex;

        };

     class DSOTracer : public AbstractFunction {

            friend class Map;

        public:
            DSOTracer(Ptr<AbstractFunction, NonNullable> parent);

            ~DSOTracer();

            void traceNewCoarse(PFrame frameToTrace, int frameGroup);

            Set<PPoint> activatePoints(int frameGroup, int pointGroup);

            int optimizeImmaturePoint(PPoint point, int minObs, int frameGroup);

            void makeNewTraces(PFrame frame);

            void makeNewTracesFrom(PFrame frame, int group);

            std::string getName() final {
                return "DSOTracer";
            }

            void viewOnReconstruction(DrawBoard &drawBoard) final {
                drawBoard.disableDepthTest();

                List<Vector3f> lastTraced, lastlastTraced;

                {
                    LockGuard lg(mLastTracedMutex);
                    lastTraced = mLastTraced;
                    lastlastTraced = mLastLastTraced;
                }

                for (auto point : mLastTraced) {

                        drawBoard.color(0,1,0);
                        drawBoard.pointSize(2);
                        drawBoard.point(point);

                }

                for (auto point : mLastLastTraced) {

                        drawBoard.color(0,0,1);
                        drawBoard.pointSize(2);
                        drawBoard.point(point);


                }

                drawBoard.enableDepthTest();

            }

            bool canShowPoint(PPoint point) final {

                return true;
                /*
                Set<PPoint> lastTraced, lastlastTraced;

                {
                    LockGuard lg(mLastTracedMutex);
                    lastTraced = mLastTraced;
                    lastlastTraced = mLastLastTraced;
                }

                if (lastTraced.count(point) > 0) {
                    return false;
                }

                if (lastlastTraced.count(point) > 0) {
                    return false;
                }

                return true;*/

            }

            DSOTracer *setPointDensity(int n) {
                mDesiredImmatureDensity.set(mDesiredImmatureDensity.i() * n / mSettingsDesiredPointDensity.i());
                mSettingsDesiredPointDensity.set(n);
                return this;
            }

         DSOTracer *setPointDensity(int desired, int immature) {
             mDesiredImmatureDensity.set(desired);
             mSettingsDesiredPointDensity.set(immature);
             return this;
         }

        protected:
            class ImmaturePointTemporaryResidual {
            public:
                DSOResidualState state_state;
                double state_energy;
                DSOResidualState state_NewState;
                double state_NewEnergy;
                OptPFrame target;
            };

            double linearizeResidual(PPoint point, float outlierTHSlack, ImmaturePointTemporaryResidual* tmpRes, float &Hdd, float &bd, float idepth);

            DSOTracerStatus trace(PFrame frame, PPoint mapPoint);

        private:
            Ptr<DSOTracerPointPrivate, NonNullable> getPrivateData(PPoint point);

            void freePrivateData(PPoint point, std::string reason);

         public:
            const int IMMATUREPOINT = getMap().createMapPointGroup("Immature DSO Tracer");

         private:
            const Pattern mPattern = PredefinedPattern::star8();

            PrivateDataInstance mPrivateDataInstance;

            PStatistic mStatisticQuality = createAveragedStatistic("Quality");
            PStatistic mStatisticPixelInterval = createAveragedStatistic("Pixel interval");
            PStatistic mStatisticDepthInterval = createAveragedStatistic("Depth interval");
            PStatistic mStatisticMinimumDistance = createStatistic("Minimum Distance");
            PStatistic mStatisticNumProposition = createStatistic("Proposition number");
            PStatistic mStatisticNumMapped = createStatistic("Mapped number");

            PStatistic mStatisticNumDeleteBecauseOutlier = createStatistic("Deleted because outlier");
            PStatistic mStatisticNumDeleteBecauseOOB = createStatistic("Deleted because OOB");


            PStatistic mStatisticNumSkippedBecauseStatus = createStatistic("Skipped because status");
            PStatistic mStatisticNumSkippedBecausePixelInterval = createStatistic("Skipped because pixel interval");
            PStatistic mStatisticNumSkippedBecauseQuality = createStatistic("Skipped because quality");
            PStatistic mStatisticNumSkippedBecauseDepth = createStatistic("Skipped because depth");

            PStatistic mStatisticNumNonMapped = createStatistic("Non Mapped");
            PStatistic mStatisticNumDropped = createStatistic("Dropped");

            Features::PixelSelector *mPixelSelector = nullptr;

            scalar_t mCurrentMinimumDistance = 2;

            Mutex mLastTracedMutex;
            List<Vector3f> mLastTraced, mLastLastTraced;

            Parameter mDesiredImmatureDensity = createParameter("immatureDensity", 600);
            Parameter mSettingsDesiredPointDensity = createParameter("desiredPointDensity", 800);
            Parameter mMinIDepthHAct = createParameter("Min iDepth H Act", 100.0f);
            Parameter mGNItsOnPointActation = createParameter("Iteration on point activation", 3);
            Parameter mSettingHuberTH = createParameter("Huber Threshold", 9.0f);
            Parameter mSettingOutlierTH = createParameter("Outlier threshold", 12.0f * 12.0f);
            Parameter mSettingOutlierTHSumComponent = createParameter("outlierTHSumComponent", 50.0f * 50.0f);
            Parameter mSettingMaxPixSearch = createParameter("Max pixel search", 0.027f);
            Parameter mSettingMaxSlackInterval = createParameter("Max slack interval", 1.5f);
            Parameter mSettingTraceSetpSize = createParameter("Intial Step size", 1.0f);
            Parameter mSettingTraceMinImprovementFactor = createParameter("Minimum improvement factor", 2.0f);
            Parameter mMinTraceTestRadius = createParameter("Trace test radius", 2.0f);
            Parameter mSettingsExtraSlackOnTH = createParameter("ExtraSlackOnTH", 1.2f);
            Parameter mSettingsMinTraceQuality = createParameter("Min Trace Quality", 3.0f);

        };

    }


}

#endif //CML_PHOTOMETRIC_H
