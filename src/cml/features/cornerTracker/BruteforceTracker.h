#ifndef CML_BRUTEFORCETRACKER_H
#define CML_BRUTEFORCETRACKER_H

#include <cml/config.h>
#include <cml/features/cornerTracker/AbstractCornerTracker.h>
#include <cml/utils/KDTree.h>
#include <cml/features/Features.h>

namespace CML::Features {

    template <typename Descriptor> class BruteforceTracker : public AbstractCornerTracker<Descriptor> {

    public:
        BruteforceTracker(Ptr<AbstractFunction, Nullable> parent) : AbstractCornerTracker<Descriptor>(parent) {

        }

        std::string getName() override {
            return "Bruteforce Tracker";
        }

        List<Matching> compute(Map &map, PFrame frameA, int groupA, const List<Descriptor> &descriptorsA, PFrame frameB, const int groupB, const List<Descriptor> &descriptorsB) {

            auto cornersA = frameA->getFeaturePoints(groupA);
            auto cornersB = frameA->getFeaturePoints(groupB);


            List<Matching> result;

            List<List<Pair<scalar_t, int>>> allPairs;
            allPairs.resize(descriptorsA.size());

            #if CML_USE_OPENMP
            #pragma omp  for
            #endif
            for (int i = 0; i < descriptorsA.size(); i++) {
                allPairs[i].resize(descriptorsB.size());
                for (int j = 0; j < descriptorsB.size(); j++) {
                    allPairs[i][j] = Pair<scalar_t, scalar_t>(descriptorsA[i].distance(descriptorsB[j]), j);
                }
            }

            for (int i = 0; i < descriptorsA.size(); i++) {

                std::partial_sort(allPairs[i].begin(),
                                  allPairs[i].begin() + 2,
                                  allPairs[i].end());

                if (allPairs[i][0].first / allPairs[i][1].first < 0.75) {
                    result.emplace_back(Matching(allPairs[i][0].first, frameA, frameB, FeatureIndex(groupA, i), FeatureIndex(groupB, allPairs[i][0].second)));
                }

            }


            this->setLastResult(frameA, cornersA, frameB, cornersB, result);

            return result;

        }

    };

}

#endif