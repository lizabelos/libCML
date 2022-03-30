#ifndef CML_REPROJECTIONTRACKER_H
#define CML_REPROJECTIONTRACKER_H

#include <cml/config.h>
#include <cml/features/cornerTracker/AbstractCornerTracker.h>
#include <cml/utils/KDTree.h>

namespace CML::Features {

    template <typename Descriptor> class ReprojectionTracker : public AbstractFunction {

    public:
        ReprojectionTracker(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent) {

        }

        std::string getName() override {
            return "Reprojection Tracker";
        }

        inline List<Matching> compute(PFrame frameA, int groupA, const List<Descriptor> &descriptorsA, PFrame frameB, int groupB, const List<Descriptor> &descriptorsB, int mapPointGroup, int radius = -1, List<Matching> initialMatching = List<Matching>()) {

            // float threshold = mThreshold.f() * (float)Descriptor::L * 8.0f;

            List<NearestNeighbor> nearestNeighbor;

            auto cornersA = frameA->getFeaturePoints(groupA);
            auto cornersB = frameA->getFeaturePoints(groupB);


            this->getTimer().start();

            Set<PPoint> toExclude;
            for (auto m : initialMatching) {
                if (m.getMapPoint().isNotNull()) {
                    toExclude.insert(m.getMapPoint());
                }
            }

            float r = (float)(frameA->getWidth(0) + frameA->getHeight(0)) * mRadius.f() / 2.0f;


            CornerMatchingGraph matching(std::max(cornersA.size(), cornersB.size()));
            for (size_t i = 0; i < cornersB.size(); i++) {

                OptPPoint mapPoint = frameB->getMapPoint(FeatureIndex(groupB, i));

                if (mapPoint.isNull()) {
                    continue;
                }


                if (toExclude.count(mapPoint) > 0) {
                    continue;
                }

                if (mapPoint->isGroup(mapPointGroup) && !frameA->getFeaturePoint(mapPoint).has_value()) {

                    DistortedVector2d projection = frameA->distort(mapPoint->getWorldCoordinate().project(frameA->getCamera()), 0);

                   /* float r;
                    if (radius == -1) {
                        r = threshold * cornersB[i].processScaleFactorFromLevel();
                    } else {
                        r = radius;
                    }*/

                    frameA->processNearestNeighborsInRadius(groupA, projection, r, nearestNeighbor);

                    for (NearestNeighbor nn : nearestNeighbor) {
                        matching.set(nn.index, i, descriptorsB[i].distance(descriptorsA[nn.index]));
                    }

                }



            }

            List<Matching> result = CornerMatchingGraph::toMatching(matching.resolveByRatio(mRatio.f()), frameA, groupA, frameB, groupB);
            List<Matching> filtered;

            for (auto m : result) {
                    filtered.emplace_back(m);
                    initialMatching.emplace_back(m);

            }

            // List<Matching> result = matching.matchingListMad(frameB, frameA, 1.0f);


            this->getTimer().stop();
            this->setLastResult(frameA, cornersA, frameB, cornersB, filtered);

            return initialMatching;

        }

        inline List<Pair<int, int>> compute(PFrame frameA, int groupA, const List<Descriptor> &descriptorsA, List<PPoint> mapPoints, const List<Descriptor> &descriptorsB, Optional<Camera> camera = Optional<Camera>()) {

            if (!camera.has_value()) {
                camera = frameA->getCamera();
            }

            List<NearestNeighbor> nearestNeighbor;

            float threshold = mThreshold.f() * (float)Descriptor::L * 8.0f;

            auto cornersA = frameA->getFeaturePoints(groupA);

            //float r = (float)(frameA->getWidth(0) + frameA->getHeight(0)) * mRadius.f() / 2.0f;

            this->getTimer().start();

            CornerMatchingGraph matching(std::max(cornersA.size(), mapPoints.size()));
            for (int i = 0; i < mapPoints.size(); i++) {

                auto mapPoint = mapPoints[i];

                DistortedVector2d projection = frameA->distort(mapPoint->getWorldCoordinate().project(camera.value()), 0);

                //Corner originalCorner = mapPoint->getReferenceCorner();
                //scalar_t scaleFactor = originalCorner.processScaleFactorFromLevel();

               // List<NearestNeighbor> nearestNeighbor = frameA->processNearestNeighborsInRadius(groupA, projection, r);

               frameA->processNearestNeighbors(groupA, projection, 5, nearestNeighbor);


                for (NearestNeighbor nn : nearestNeighbor) {

                    auto corner = cornersA[nn.index];

                    matching.set(nn.index, i, descriptorsB[i].distance(descriptorsA[nn.index]));
                }

            }

            List<CornerMatchingGraph::match2_t> result = matching.resolveByRatio(mRatio.f());
            List<Pair<int, int>> filtered;
            List<Vector2> lastMatchings;

            for (auto m : result) {
                if (m.score < threshold) { // todo : check the scale factor of both points
                    filtered.emplace_back(m.indexA, m.indexB);
                    lastMatchings.emplace_back(cornersA[m.indexA].point0());
                }
            }

            this->getTimer().stop();
            //this->setLastResult(filtered);

            LockGuard lg(mLastMatchingsMutex);
            mLastMatchings = lastMatchings;

            return filtered;

        }

        ReprojectionTracker<Descriptor> *setThreshold(float value) {
            mThreshold.set(value);
            return this;
        }

        ReprojectionTracker<Descriptor> *setRatio(float value) {
            mRatio.set(value);
            return this;
        }

        void setColor(float r, float g, float b) {
            mColor = Vector3(r,g,b);
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) override {
            /*List<Corner> corners;
            if (frame == mLastFrameA) {
                corners = mLastCornersA;
            } else if (frame == mLastFrameB) {
                corners = mLastCornersB;
            } else {
                return;
            }*/
            List<Vector2> lastMatching;

            {
                LockGuard lg(mLastMatchingsMutex);
                lastMatching = mLastMatchings;
            }

            drawBoard.pointSize(3);
            drawBoard.color(mColor(0), mColor(1), mColor(2));

            for (const Vector2 &pos : lastMatching) {
                drawBoard.point((Eigen::Vector2f)pos.cast<float>());
            }

        }

    private:
        Parameter mThreshold = this->createParameter("Threshold", 0.1f);
        Parameter mRatio = this->createParameter("Ratio", 0.9f);
        Parameter mRadius = this->createParameter("Radius", 0.1f);

        Vector3 mColor = Vector3(1, 1, 0);

        Mutex mLastMatchingsMutex;
        List<Vector2> mLastMatchings;
    };

}

#endif