#ifndef CML_RADIUSTRACKER_H
#define CML_RADIUSTRACKER_H

#include <cml/config.h>
#include <cml/features/cornerTracker/AbstractCornerTracker.h>
#include <cml/utils/KDTree.h>
#include <cml/features/cornerTracker/DefaultParameters.h>

namespace CML::Features {

    template <typename Descriptor> class RadiusTracker : public AbstractCornerTracker<Descriptor> {

    public:
        RadiusTracker(Ptr<AbstractFunction, Nullable> parent) : AbstractCornerTracker<Descriptor>(parent) {

        }

        std::string getName() final {
            return "Radius Tracker";
        }

        RadiusTracker *setRadius(float radius) {
            mWindowSize.set(radius);
            return this;
        }

        RadiusTracker *setRatio(float ratio) {
            mRatio.set(ratio);
            return this;
        }

        inline List<Matching> compute(PFrame frameToTrack, int frameToTrackGroup, const List<Descriptor> &frameToTrackDescriptors, PFrame referenceFrame, int referenceFrameGroup, const List<Descriptor> &referenceFrameDescriptors, bool onlyMapped = false) {

            this->getTimer().start();

            auto referenceCorners = referenceFrame->getFeaturePoints(referenceFrameGroup);
            auto frameToTrackCorners = frameToTrack->getFeaturePoints(frameToTrackGroup);

            if (mLastSeenFrame != referenceFrame) {
                mLastSeenFrame = referenceFrame;
                mLastSeenCorners = referenceCorners;
            }

            float windowSize = mWindowSize.f() * Vector2(referenceFrame->getWidth(0), referenceFrame->getHeight(0)).norm();

            CornerMatchingGraph graph(std::max(referenceCorners.size(), frameToTrackCorners.size()));
            for (size_t i = 0; i < referenceCorners.size(); i++) {
                if (onlyMapped && referenceFrame->getMapPoint(FeatureIndex(referenceFrameGroup, i)).isNotNull()) {
                    continue;
                }
                if (mLastSeenCorners[i].level() > mLevelFilter.i()) {
                    continue;
                }
                List<NearestNeighbor> nearestNeighbor = frameToTrack->processNearestNeighborsInRadius(frameToTrackGroup, mLastSeenCorners[i].point(0), windowSize);
                for (NearestNeighbor nn : nearestNeighbor) {
                    int distance = referenceFrameDescriptors[i].distance(frameToTrackDescriptors[nn.index]);
                    if (mFilterThresholdAfter.b()) {
                        graph.set(nn.index, i, referenceFrameDescriptors[i].distance(frameToTrackDescriptors[nn.index]));
                    } else {
                        if (distance < mThreshold.f() * (float) Descriptor::NUMELEMENTS) {
                            graph.set(nn.index, i, referenceFrameDescriptors[i].distance(frameToTrackDescriptors[nn.index]));
                        }
                    }
                }
            }


            List<Matching> matchings = CornerMatchingGraph::toMatching(graph.resolveByRatio(mRatio.f()), frameToTrack, frameToTrackGroup, referenceFrame, referenceFrameGroup);

            if (mFilterThresholdAfter.b()) {
                List<Matching> results;
                float threshold = mThreshold.f() * (float) Descriptor::NUMELEMENTS;
                for (const Matching &matching : matchings) {
                    if (matching.getDescriptorDistance() < threshold) {
                        results.emplace_back(matching);
                    }
                }
                matchings = results;
            }

            matchings = checkMathingsRotation(matchings);

            for (auto matching : matchings) {
                mLastSeenCorners[matching.getIndexB(referenceFrame).index] = matching.getFeaturePointA(frameToTrack);
            }

            this->setLastResult(frameToTrack, frameToTrackCorners, referenceFrame, referenceCorners, matchings);

            this->getTimer().stop();

            return matchings;

        }

    private:
        Parameter mWindowSize = this->createParameter("Radius", 0.08f);
        Parameter mThreshold = this->createParameter("Threshold", DefaultParameters::threshold);
        Parameter mRatio = this->createParameter("Ratio", DefaultParameters::ratio);
        Parameter mFilterThresholdAfter = this->createParameter("Filter threshold after", true);
        Parameter mLevelFilter = this->createParameter("Level filter", 0);

        OptPFrame mLastSeenFrame;
        List<Corner> mLastSeenCorners;

    };

}

#endif