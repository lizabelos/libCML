#ifndef CML_DSOCONTEXT_H
#define CML_DSOCONTEXT_H

#include <cml/config.h>
#include <cml/optimization/dso/DSOTypes.h>
#include <cml/optimization/dso/DSOPoint.h>
#include <cml/optimization/dso/DSOFrame.h>
#include <cml/map/MapObject.h>
#include <cml/map/Frame.h>
#include <cml/base/AbstractFunction.h>

namespace CML {

    namespace Optimization {

        class DSOPoint;

        class DSOFrame;


        typedef enum LightModeOptimization {
            OptimizeLightAB, OptimizeLightA, OptimizeLightB, OptimizeNoLight
        }LightModeOptimization;

        class DSOContext {

        public:
            DSOContext(Map &map) : ACTIVEKEYFRAME(map.createFrameGroup("DSO Active Key Frame")), ACTIVEPOINT(map.createMapPointGroup("DSO Active Point")) {
                mPointPrivateDataInstance = map.getMapPointsPrivataDataContext().createInstance();
                mFramePrivateDataInstance = map.getFramePrivataDataContext().createInstance();
                mFrameKeyCounter = 0;
            }

            Ptr<DSOPoint, NonNullable> get(PPoint point, bool newPoint = false) {
                bool isNew;
                auto pointData = point->getPrivate().get<DSOPoint>(mPointPrivateDataInstance, isNew);
                assertThrow(newPoint == isNew, "DSO Bundle Adjustment point management bug");
                return pointData;
            }

            Ptr<DSOPoint, NonNullable> unsafe_get(PPoint point) {
                return point->getPrivate().unsafe_get<DSOPoint>(mPointPrivateDataInstance);
            }

            bool have(PPoint point) {
                return point->getPrivate().have(mPointPrivateDataInstance);
            }

            Ptr<DSOFrame, NonNullable> get(PFrame frame) {
                bool isNew;
                auto data = frame->getPrivateData().get<DSOFrame>(mFramePrivateDataInstance, isNew);
                if (isNew) {
                    data->ab_exposure = frame->getExposure().getExposureFromCamera();
                    data->keyid = mFrameKeyCounter++;
                }
                return data;
            }

            void free(PPoint point) {
                point->getPrivate().free(mPointPrivateDataInstance, "DSOContext::free");
            }

            const Set<PPoint, Hasher> &getPoints() {
                return mPoints;
            }

            List<Pair<PPoint, Ptr<DSOPoint, NonNullable>>> getPointsAsList() {
                List<Pair<PPoint, Ptr<DSOPoint, NonNullable>>> result;
                result.reserve(mPoints.size());
                for (auto point : mPoints) {
                    result.emplace_back(point, get(point));
                }
                return result;
            }

            void addPoint(PPoint point) {
                assertThrow(haveFrame(point->getReferenceFrame()), "The reference frame of the point is not in the context");
                // assertThrow(mImmaturePoints.count(point) == 0, "You need to remove the immature point first, before adding it");
                mPoints.insert(point);
                get(point->getReferenceFrame())->points.insert(point);

                for (auto frame : getFrames()) {
                    frame->addDirectApparitions(point);
                }
                point->setGroup(ACTIVEPOINT, true);

                auto pointData = get(point, true);
                for (size_t idx = 0; idx < PredefinedPattern::star8().size(); idx++) {
                    Vector2 shift = PredefinedPattern::star8(idx);
                    scalar_t refColor = point->getGrayPatch(shift.x(), shift.y(), 0);
                    pointData->colors[idx] = refColor;
                }
            }

            void removePoint(PPoint point, bool marginalize = false) {
                if (mPoints.count(point) == 0) {
                    return;
                }
                Set<DSOResidual*> residualsToRemove = get(point)->residuals; // Important to make a copy, and not pass as reference
                if (marginalize) {
                    Set<PFrame, Hasher> frames;
                    for (auto residual : residualsToRemove) {
                        frames.insert(residual->elements.frame);
                    }
                    for (auto frame : frames) {
                        get(frame)->numMarginalized++;
                    }
                }
                removeResiduals(residualsToRemove);

                assertThrow(mPoints.count(point) == 0, "Remove point seems to have a problem");
            }

            void removePoints(const Set<PPoint, Hasher> &points) {
                Set<DSOResidual*> residualsToRemove;
                for (auto point : points) {
                    Set<DSOResidual*> residualsToRemoveForPoint = get(point)->residuals;
                    residualsToRemove.insert(residualsToRemoveForPoint.begin(), residualsToRemoveForPoint.end());
                }
                removeResiduals(residualsToRemove);
                //for (auto point : points) {
                //    assertThrow(mPoints.count(point) == 0, "Remove points seems to have a problem");
                //}
            }

            const List<PFrame> &getFrames() {
                return mFrames;
            }

            void addFrame(PFrame frameToInsert) {
                if (haveFrame(frameToInsert)) {
                    logger.warn("Frame already inserted");
                    return;
                }
                for (auto frame : mFrames) {
                    if (frameToInsert->getId() < frame->getId()) {
                        abort();
                    }
                }
                mFrames.emplace_back(frameToInsert);
                makeFrameId();

                for (auto point : getPoints()) {
                    frameToInsert->addDirectApparitions(point);
                }

                frameToInsert->setGroup(ACTIVEKEYFRAME, true);

            }

            void removeFrame(PFrame frameToRemove) {
                Set<PPoint, Hasher> pointsToRemove = get(frameToRemove)->points;
                removePoints(pointsToRemove);
                Set<DSOResidual*> residualsToRemove = get(frameToRemove)->residuals;
                removeResiduals(residualsToRemove);
                assertThrow(get(frameToRemove)->points.size() == 0, "Not all the points have been removed for the frame");
                assertThrow(get(frameToRemove)->residuals.size() == 0, "Not all the residuals have been removed for the frame");
                List<PFrame> newFrames;
                for (auto frame : mFrames) {
                    if (frame != frameToRemove) {
                        newFrames.emplace_back(frame);
                    }
                }
                mFrames = newFrames;
                makeFrameId();

                frameToRemove->setGroup(ACTIVEKEYFRAME, false);

            }

            bool haveFrame(PFrame frameToSearch) {
                for (auto frame : mFrames) {
                    if (frame == frameToSearch) {
                        return true;
                    }
                }
                return false;
            }

            void makeFrameId() {
                for (size_t i = 0; i < getFrames().size(); i++) {
                    get(getFrames()[i])->id = i;
                }
            }

            const Set<DSOResidual*> &getResiduals() {
                return mResiduals;
            }

            void addResidual(DSOResidual* residualToInsert) {
                assertThrow(haveFrame(residualToInsert->elements.mapPoint->getReferenceFrame()), "The reference frame of the residual is not in the context");
                assertThrow(haveFrame(residualToInsert->elements.frame), "The frame of the residual is not in the context");
                assertThrow(mPoints.count(residualToInsert->elements.mapPoint) > 0, "The point of the residual is not in the context");
                mResiduals.insert(residualToInsert);
                get(residualToInsert->elements.mapPoint)->residuals.insert(residualToInsert);
                get(residualToInsert->elements.frame)->residuals.insert(residualToInsert);
            }

            Set<PPoint, Hasher> removeResiduals(const Set<DSOResidual*> &residualsToRemove) {
                for (auto residualToRemove : residualsToRemove) {
                    mResiduals.erase(residualToRemove);
                    get(residualToRemove->elements.mapPoint)->residuals.erase(residualToRemove);
                    get(residualToRemove->elements.frame)->residuals.erase(residualToRemove);
                    get(residualToRemove->elements.frame)->numResidualsOut++;
                    freeResidual(residualToRemove);
                }
                return removeAllPointsWithoutResidual();

            }

        protected:
            Set<PPoint, Hasher> removeAllPointsWithoutResidual() {
                Set<PPoint, Hasher> pointsToRemove;
                for (auto point : mPoints) {
                    if (get(point)->residuals.empty()) pointsToRemove.insert(point);
                }
                for (auto point : pointsToRemove) {
                    point->setGroup(ACTIVEPOINT, false);
                    mPoints.erase(point);
                    get(point->getReferenceFrame())->points.erase(point);
                }
                return pointsToRemove;
            }

            void freeResidual(DSOResidual* r) {
                delete r;
            }

        private:
            Atomic<size_t> mFrameKeyCounter;

            PrivateDataInstance mPointPrivateDataInstance, mFramePrivateDataInstance;

            Set<PPoint, Hasher> mPoints;
            List<PFrame> mFrames;

            Set<DSOResidual*> mResiduals;

        public:
            const int ACTIVEKEYFRAME;
            const int ACTIVEPOINT;


        };

    }

}

#endif