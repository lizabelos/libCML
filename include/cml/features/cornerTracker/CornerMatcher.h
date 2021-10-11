#ifndef CML_CORNER_MATCHER_H
#define CML_CORNER_MATCHER_H

#include <unordered_map>

#include "cml/config.h"
#include "cml/base/AbstractFunction.h"
#include "cml/map/Frame.h"
#include "cml/map/MapObject.h"

namespace CML {

    class Matching {
    public:
        Matching(scalar_t descriptorDistance, PFrame frameA, PFrame frameB, FeatureIndex indexA, FeatureIndex indexB) :
        mFrameA(frameA), mFrameB(frameB), mIndexA(indexA), mIndexB(indexB), mDescriptorDistance(descriptorDistance)
        {
        }

        inline Corner getFeaturePointA(PFrame frame) const {
            assertThrow(mFrameA == frame, "You probably switched frameA and frameB by mistake");
            return mFrameA->getFeaturePoint(mIndexA);
        }

        inline Corner getFeaturePointB(PFrame frame) const {
            assertThrow(mFrameB == frame, "You probably switched frameA and frameB by mistake");
            return mFrameB->getFeaturePoint(mIndexB);
        }

        inline Corner getFeaturePoint(PFrame frame) const {
            if (mFrameA == frame) {
                return getFeaturePointA(frame);
            }
            if (mFrameB == frame) {
                return getFeaturePointB(frame);
            }
            abort();
        }

        inline Corner noAssertGetFeaturePointA() const {
            return mFrameA->getFeaturePoint(mIndexA);
        }

        inline Corner noAssertGetFeaturePointB() const {
            return mFrameB->getFeaturePoint(mIndexB);
        }

        inline OptPPoint getMapPoint() const {
            if (mFrameA->getMapPoint(mIndexA).isNotNull()) {
                return mFrameA->getMapPoint(mIndexA);
            }
            return mFrameB->getMapPoint(mIndexB);
        }

        inline OptPPoint getMapPointA() const {
            return mFrameA->getMapPoint(mIndexA);
        }

        inline OptPPoint getMapPointB() const {
            return mFrameB->getMapPoint(mIndexB);
        }

        inline UndistortedVector2d getUndistortedA(PFrame frame, int level) const {
            return mFrameA->undistort(getFeaturePointA(frame).point(level), level);
        }

        inline UndistortedVector2d getUndistortedB(PFrame frame, int level) const {
            return mFrameB->undistort(getFeaturePointB(frame).point(level), level);
        }

        inline UndistortedVector2d noAssertGetUndistortedA(int level) const {
            return mFrameA->undistort(noAssertGetFeaturePointA().point(level), level);
        }

        inline UndistortedVector2d noAssertGetUndistortedB(int level) const {
            return mFrameB->undistort(noAssertGetFeaturePointB().point(level), level);
        }

        inline PFrame getFrameA() const {
            return mFrameA;
        }

        inline PFrame getFrameB() const {
            return mFrameB;
        }

        inline FeatureIndex getIndexA(PFrame frame) const {
            assertThrow(mFrameA == frame, "You probably switched frameA and frameB by mistake");
            return mIndexA;
        }

        inline FeatureIndex getIndexB(PFrame frame) const {
            assertThrow(mFrameB == frame, "You probably switched frameA and frameB by mistake");
            return mIndexB;
        }

        inline FeatureIndex noAssertGetIndexA() const {
            return mIndexA;
        }

        inline FeatureIndex noAssertGetIndexB() const {
            return mIndexB;
        }

        inline scalar_t getDescriptorDistance() const {
            return mDescriptorDistance;
        }

    private:
        PFrame mFrameA, mFrameB;
        FeatureIndex mIndexA, mIndexB;
        scalar_t mDescriptorDistance;

    };

    class CornerMatchingFilter {

    public:
        virtual List<Matching> filter(const List<Corner> &cornersA, const List<Corner> &cornersB, List<Matching> &matchings) const = 0;

    };

    inline List<Matching> filterMatchingsByGroup(const List<Matching> &input, int group) {
        List<Matching> output;
        for (const Matching matching : input) {
            if (matching.getMapPoint().isNotNull() && matching.getMapPoint()->isGroup(group)) {
                output.emplace_back(matching);
            }
        }
        return output;
    }

    inline List<Matching> checkMathingsRotation(const List<Matching> &input) {
        const int L = 30;
        const float factor = 1.0f/(float)L;
        List<int> histo[L];

        for (size_t i = 0; i < input.size(); i++) {
            const auto &m = input[i];
            float rot = m.noAssertGetFeaturePointA().angle() - m.noAssertGetFeaturePointB().angle();
            if(rot<0.0) {
                rot += 360.0f;
            }
            int bin = round(rot * factor);
            if(bin == L) {
                bin = 0;
            }
            assert(bin>=0 && bin < L);
            histo[bin].push_back(i);
        }

        int max1=0;
        int max2=0;
        int max3=0;

        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }

        List<bool> toKeep;
        toKeep.resize(input.size(), true);

        for(int i=0; i<L; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3) {
                continue;
            }
            for(size_t j=0; j<histo[i].size(); j++)
            {
                int idx = histo[i][j];
                toKeep[idx] = false;
            }
        }

        List<Matching> result;
        int numRemoved = 0;
        for (size_t i = 0; i < input.size(); i++) {
            if (toKeep[i]) {
                result.emplace_back(input[i]);
            } else {
                numRemoved++;
            }
        }

        logger.debug("By checking the rotation, we removed " + std::to_string(numRemoved) + " matchings");

        return result;
    }

    inline List<Pair<PFrame, PPoint>> filterMatchingsByGroupToPairs(const List<Matching> &input, int group) {
        List<Pair<PFrame, PPoint>> output;
        for (const Matching matching : input) {
            if (matching.getMapPoint().isNotNull() && matching.getMapPoint()->isGroup(group)) {
                output.emplace_back(Pair<PFrame, PPoint>(matching.getFrameA(),matching.getMapPoint()));
            }
        }
        return output;
    }

    class CornerMatchingGraph {

    public:
        typedef struct match_t {

            match_t(size_t index, float score) {
                this->index = index;
                this->score = score;
            }

            size_t index;
            float score;
        } match_t;

        typedef struct match2_t {

            match2_t(size_t indexA, size_t indexB, float score) {
                this->indexA = indexA;
                this->indexB = indexB;
                this->score = score;
            }

            size_t indexA, indexB;
            float score;
        } match2_t;

    public:
        CornerMatchingGraph(size_t size) {
            mAtoB.resize(size);
            mBtoA.resize(size);
            mPointA.resize(size);
            mPointB.resize(size);
        }

        inline void declareA(size_t a, Vector2 point) {
            assertThrow(a < mAtoB.size(), "Out of range index");
            mPointA[a] = point;
        }

        inline void declareB(size_t b, Vector2 point) {
            assertThrow(b < mAtoB.size(), "Out of range index");
            mPointB[b] = point;
        }

        inline void set(size_t a, size_t b, float score) {
            assertThrow(a < mAtoB.size(), "Out of range index");
            assertThrow(b < mBtoA.size(), "Out of range index");
            mAtoB[a][b] = score;
            mBtoA[b][a] = score;
        }

        inline void remove(size_t a, size_t b) {
            assertThrow(a < mAtoB.size(), "Out of range index");
            assertThrow(b < mBtoA.size(), "Out of range index");
            removeByValue(mAtoB[a], b);
            removeByValue(mBtoA[b], a);
        }

        List<match2_t> resolveByRatio(float ratio);

        inline void resolve(size_t a, size_t b, float score) {
            // We need a copy because we are going to modify the map
            HashMap<size_t, float> matchAtoB = mAtoB[a];
            HashMap<size_t, float> matchBtoA = mBtoA[b];

            for (auto [indexB, score] : matchAtoB) {
                remove(a, indexB);
            }

            for (auto [indexA, score] : matchBtoA) {
                remove(indexA, b);
            }

            assertEmpty(a, b);

            set(a, b, score);
            mResolved.emplace_back(match2_t(a, b, score));

        }

        inline void assertResolved(size_t a, size_t b) {
            assertThrow(a < mAtoB.size(), "Out of range index");
            assertThrow(b < mBtoA.size(), "Out of range index");
            assertThrow( mAtoB[a].size() == 1, "Not resolved");
            assertThrow(mBtoA[b].size() == 1, "Not resolved");
        }

        inline void assertEmpty(size_t a, size_t b) {
            assertThrow(a < mAtoB.size(), "Out of range index");
            assertThrow(b < mBtoA.size(), "Out of range index");
            assertThrow(mAtoB[a].empty(), "Must be empty");
            assertThrow(mBtoA[b].empty(), "Must be empty");
        }

        const List<HashMap<size_t, float>> &getAtoB() {
            return mAtoB;
        }

        const List<HashMap<size_t, float>> &getBtoA() {
            return mBtoA;
        }

        static inline List<Matching> toMatching(const List<match2_t> &matches, PFrame frameA, int groupA, PFrame frameB, int groupB) {

            List<Matching> matchings;
            matchings.reserve(matches.size());

            for (auto m : matches) {
                matchings.emplace_back(Matching(m.score, frameA, frameB, FeatureIndex(groupA, m.indexA), FeatureIndex(groupB, m.indexB)));
            }

            return matchings;

        }

       /* List<Matching> matchingList(PFrame frameA, int groupA, PFrame frameB, int groupB);

        List<match2_t> matchingList();

        List<Matching> matchingListMad(PFrame frameA, int groupA, PFrame frameB, int groupB, float mad);

        List<Matching> resolvedList(PFrame frameA, int groupA, PFrame frameB, int groupB);
*/
    private:
         static inline void removeByValue(HashMap<size_t, float> &v, size_t a) {
             v.erase(a);
            /*for(auto iter = v.begin() ; iter != v.end() ; ) {
                if ((*iter).index == a) {
                    iter = v.erase(iter);
                }
                else {
                    ++iter;
                }
            }*/
        }

        //std::unordered_map<size_t, Vector2> mPointA, mPointB;
        //std::unordered_map<size_t, std::vector<match_t>> mAtoB, mBtoA;
        List<Vector2> mPointA, mPointB;
        List<HashMap<size_t, float>> mAtoB, mBtoA;
        List<match2_t> mResolved;
    };

    inline List<Matching> regenerateMatchingVector(PFrame frameA, PFrame frameB) {

        List<Matching> matchings;

        auto mapPoints = frameA->getMapPoints();
        for (auto [indexA, mapPoint] : mapPoints) {

            FeatureIndex indexB = frameB->getIndex(mapPoint);
            if (indexB.hasValidValue()) {
                matchings.emplace_back(0, frameA, frameB, indexA, indexB);
            }

        }

        return matchings;

    }

    inline List<Matching> filterMatchingsByInliers(List<Matching> matchings, List<bool> inliers) {
        assertThrow(matchings.size() == inliers.size(), "Matchings list size is not equals to inliers list size");
        List<Matching> results;
        for (size_t i = 0; i < matchings.size(); i++) {
            if (inliers[i]) {
                results.emplace_back(matchings[i]);
            }
        }
        return results;
    }

    inline List<Matching> filterMatchingsByOutliers(List<Matching> matchings, List<bool> outliers) {
        assertThrow(matchings.size() == outliers.size(), "Matchings list size is not equals to outliers list size");
        List<Matching> results;
        for (size_t i = 0; i < matchings.size(); i++) {
            if (!outliers[i]) {
                results.emplace_back(matchings[i]);
            }
        }
        return results;
    }

}

#endif