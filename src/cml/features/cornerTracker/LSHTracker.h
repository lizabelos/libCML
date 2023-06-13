#ifndef CML_LSHTRACKER_H
#define CML_LSHTRACKER_H


#include <cml/config.h>
#include <cml/features/cornerTracker/AbstractCornerTracker.h>
#include <cml/utils/KDTree.h>
#include <cml/features/Features.h>
#include <cml/features/cornerTracker/DefaultParameters.h>

namespace CML::Features {
/*
    typedef enum LSHTrackerGoal {
        LSHTRACKERGOAL_NONE, LSHTRACKERGOAL_LOCALIZATION, LSHTRACKERGOAL_MAPPING
    } LSHTrackerGoal;

    template <int N, LSHTrackerGoal GOAL> class LSHTracker : public AbstractCornerTracker<BinaryDescriptor<N>> {

        using Descriptor = BinaryDescriptor<N>;

    public:
        LSHTracker(Ptr<AbstractFunction, Nullable> parent) : AbstractCornerTracker<Descriptor>(parent) {

        }

        std::string getName() override {
            return "LSH Tracker";
        }

        List<Matching> compute(PFrame frameA, int groupA, const List<Descriptor> &descriptorsA, PFrame frameB, const int groupB, const List<Descriptor> &descriptorsB) {

            Matrix33 F12;
            if (GOAL == LSHTRACKERGOAL_MAPPING) {
                F12 = computeF12(frameA, frameB);
            }

            auto cornersA = frameA->getFeaturePoints(groupA);
            auto cornersB = frameB->getFeaturePoints(groupB);

            this->getTimer().start();

            uint8_t allDescriptors[Descriptor::L * descriptorsB.size()];
            for (int i = 0; i < descriptorsB.size(); i++) {
                memcpy(&allDescriptors[i * Descriptor::L], descriptorsB[i].data(), Descriptor::L);
            }
            LSHTree tree(allDescriptors, descriptorsB.size(), Descriptor::L);

            List<Matching> hypothesis;

            CornerMatchingGraph matching(std::max(cornersA.size(), cornersB.size()));
            for (size_t i = 0; i < cornersA.size(); i++) {
                if (frameA->getMapPoint(FeatureIndex(groupA, i)).isNotNull() && GOAL == LSHTRACKERGOAL_MAPPING) {
                    continue;
                }
                if (frameA->getMapPoint(FeatureIndex(groupA, i)).isNull() && GOAL == LSHTRACKERGOAL_LOCALIZATION) {
                    continue;
                }
                List<NearestNeighbor> nearestNeighbor = tree.getNearestNeighbors(descriptorsA[i].data(), 3);
                for (auto neighbor : nearestNeighbor) {
                    if (neighbor.distance < mThreshold.f() * (float) Descriptor::L * 8.0f) {
                        if (GOAL == LSHTRACKERGOAL_LOCALIZATION || GOAL == LSHTRACKERGOAL_NONE || checkDistEpipolarLine(cornersA[i], cornersB[neighbor.index], F12)) {
                            hypothesis.emplace_back(Matching(neighbor.distance, frameA, frameB, FeatureIndex(groupA, i),FeatureIndex(groupB, neighbor.index)));
                        }
                    }
                }
            }

            for (const Matching &match : hypothesis) {
                matching.set(match.getIndexA(frameA).index, match.getIndexB(frameB).index, match.getDescriptorDistance());
            }

            List<Matching> result = CornerMatchingGraph::toMatching(matching.resolveByRatio(mRatio.f()), frameA, groupA, frameB, groupB);

            this->getTimer().stop();

            this->setLastResult(frameA, cornersA, frameB, cornersB, result);

            return result;
            //return matching.matchingListMad(frameB, frameA, 1.0f);

        }

        void setRatio(float value) {
            mRatio.set(value);
        }

    protected:
        bool checkDistEpipolarLine(const Corner &kp1, const Corner &kp2, const Matrix33 &F12) {
            // Epipolar line in second image l = x1'F12 = [a b c]
            const float a = kp1.x() * F12(0, 0) + kp1.y() * F12(1, 0) + F12(2, 0);
            const float b = kp1.x() * F12(0, 1) + kp1.y() * F12(1, 1) + F12(2, 1);
            const float c = kp1.x() * F12(0, 2) + kp1.y() * F12(1, 2) + F12(2, 2);

            const float num = a * kp2.x() + b * kp2.y() + c;

            const float den = a * a + b * b;

            if (den == 0)
                return false;

            const float dsqr = num * num / den;

            return dsqr < 3.84;
            // return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave]; // todo
        }

        CML::Matrix33 computeF12(PFrame pKF1, PFrame pKF2) {
            Matrix33 R1w = pKF1->getCamera().getRotationMatrix();
            Vector3 t1w = pKF1->getCamera().getTranslation();
            Matrix33 R2w = pKF2->getCamera().getRotationMatrix();
            Vector3 t2w = pKF2->getCamera().getTranslation();

            Matrix33 R12 = R1w * R2w.transpose();
            Vector3 t12 = -R1w * R2w.transpose() * t2w + t1w;

            Matrix33 t12x = skew(t12);

            Matrix33 K1 = pKF1->getK(0);
            Matrix33 K2 = pKF2->getK(0);


            return K1.transpose().inverse() * t12x * R12 * K2.inverse();
        }

    private:
        Parameter mThreshold = this->createParameter("Threshold", DefaultParameters::threshold);
        Parameter mRatio = this->createParameter("Ratio", DefaultParameters::ratio);

    };
*/
}

#endif