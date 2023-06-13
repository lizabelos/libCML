#include "cml/config.h"
#include "cml/features/cornerTracker/CornerMatcher.h"
#include "cml/maths/Utils.h"


CML::List<CML::CornerMatchingGraph::match2_t> CML::CornerMatchingGraph::resolveByRatio(float ratio) {

    List<match2_t> toResolveA, toResolveB, finalResult;

    toResolveA.reserve(mAtoB.size());
    toResolveB.reserve(mAtoB.size());
    finalResult.reserve(mAtoB.size());

    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];

        if (matchB.size() != 1) {
            continue;
        }

        auto [indexB, score] = *matchB.begin();

        if (mBtoA[indexB].size() != 1) {
            continue;
        }

        finalResult.emplace_back(match2_t(indexA, indexB, score));

    }

    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];

        if (matchB.size() <= 1) {
            continue;
        }

        size_t bestIndexesB[2];
        float bestScores[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};

        for (auto [indexB, score] : matchB) {
            if (score < bestScores[0]) {
                bestScores[1] = bestScores[0];
                bestIndexesB[1] = bestIndexesB[0];
                bestScores[0] = score;
                bestIndexesB[0] = indexB;
            } else if (score < bestScores[1]) {
                bestScores[1] = score;
                bestIndexesB[1] = indexB;
            }
        }

        if (bestScores[0]  < bestScores[1] * ratio) {
            toResolveA.emplace_back((match2_t){indexA, bestIndexesB[0], bestScores[0]});
        }

    }

    for (size_t indexB = 0; indexB < mBtoA.size(); indexB++) {
        const auto &matchA = mBtoA[indexB];

        if (matchA.size() <= 1) {
            continue;
        }

        size_t bestIndexesA[2];
        float bestScores[2] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};

        for (auto [indexA, score] : matchA) {
            if (score < bestScores[0]) {
                bestScores[1] = bestScores[0];
                bestIndexesA[1] = bestIndexesA[0];
                bestScores[0] = score;
                bestIndexesA[0] = indexA;
            } else if (score < bestScores[1]) {
                bestScores[1] = score;
                bestIndexesA[1] = indexA;
            }
        }

        if (bestScores[0] < bestScores[1] * ratio) {
            toResolveB.emplace_back((match2_t){bestIndexesA[0], indexB, bestScores[0]});
        }

    }


    for (auto matchA : toResolveA) {

        bool isOk = true;
        for (auto matchB : toResolveB) {
            if (matchB.indexA == matchA.indexA) {
                if (matchB.indexB != matchA.indexB) isOk = false;
            }
            if (matchB.indexB == matchA.indexB) {
                if (matchB.indexA != matchA.indexA) isOk = false;
            }
        }

        if (isOk) {
            finalResult.emplace_back((match2_t){matchA.indexA, matchA.indexB, matchA.score});
            // resolve(matchA.indexA, matchA.indexB, matchA.score);
            // assertResolved(matchA.indexA, matchA.indexB);
            // resolved++;
        }
    }

    for (auto matchA : toResolveB) {

        bool isOk = true;
        for (auto matchB : toResolveA) {
            if (matchB.indexA == matchA.indexA) {
                if (matchB.indexB != matchA.indexB) isOk = false;
            }
            if (matchB.indexB == matchA.indexB) {
                if (matchB.indexA != matchA.indexA) isOk = false;
            }
        }

        if (isOk) {
            finalResult.emplace_back((match2_t){matchA.indexA, matchA.indexB, matchA.score});
            //resolve(matchA.indexA, matchA.indexB, matchA.score);
            //assertResolved(matchA.indexA, matchA.indexB);
            //resolved++;
        }
    }

    //resolve(finalResult);

    return finalResult;

}


/*
CML::List<CML::Matching> CML::CornerMatchingGraph::matchingList(PFrame frameA, int groupA, PFrame frameB, int groupB) {

    List<Matching> matchings;

    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];

        if (matchB.size() != 1) {
            continue;
        }

        auto [indexB, score] = *matchB.begin();

        if (mBtoA[indexB].size() != 1) {
            continue;
        }

        matchings.emplace_back(Matching(score, frameA, frameB, FeatureIndex(groupA, indexA), FeatureIndex(groupB, indexB)));

    }

    return matchings;

}

CML::List<CML::CornerMatchingGraph::match2_t> CML::CornerMatchingGraph::matchingList() {

    List<match2_t> matchings;

    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];

        if (matchB.size() != 1) {
            continue;
        }

        auto [indexB, score] = *matchB.begin();

        if (mBtoA[indexB].size() != 1) {
            continue;
        }

        matchings.emplace_back(match2_t(indexA, indexB, score));

    }

    return matchings;

}

CML::List<CML::Matching> CML::CornerMatchingGraph::resolvedList(PFrame frameA, int groupA, PFrame frameB, int groupB) {

    List<Matching> matchings;

    for (auto match : mResolved) {

        if (mAtoB[match.indexA].size() != 1) {
            continue;
        }

        if (mBtoA[match.indexB].size() != 1) {
            continue;
        }

        matchings.emplace_back(Matching(match.score, frameA, frameB, FeatureIndex(groupA, match.indexA), FeatureIndex(groupB, match.indexB)));

    }

    return matchings;

}

CML::List<CML::Matching>
CML::CornerMatchingGraph::matchingListMad(PFrame frameA, int groupA, PFrame frameB, int groupB, float madValue) {

    List<Matching> matchings;

    List<float> scores;
    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];
        if (matchB.size() != 1) {
            continue;
        }
        auto [indexB, score] = *matchB.begin();
        scores.emplace_back(score);
    }

    if (scores.empty()) {
        return List<Matching>();
    }

    float threshold = median(scores) + madValue * mad(scores);

    for (size_t indexA = 0; indexA < mAtoB.size(); indexA++) {
        const auto &matchB = mAtoB[indexA];

        if (matchB.size() != 1) {
            continue;
        }

        auto [indexB, score] = *matchB.begin();

        if (score > threshold) {
            continue;
        }

        matchings.emplace_back(Matching(score, frameA, frameB, FeatureIndex(groupA, indexA), FeatureIndex(groupB, indexB)));

    }

    return matchings;

}
*/