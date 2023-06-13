#include "cml/features/bow/Relocalization.h"

void CML::Features::Relocalization::onFrameChangeGroup(Map &map, PFrame frame, int groupId, bool state) {
    if (groupId == LOOPCLOSUREFRAMEGROUP && state == true) {
        for (int i = 0; i < frame->numFeaturesGroup(); i++) if (frame->haveBoW(i)) {
            for (auto vit = frame->getBoW(i)->bowVec.begin(), vend = frame->getBoW(i)->bowVec.end(); vit != vend; vit++) {
                mInvertedFile[vit->first].emplace_back(frame);
            }
        }
    }
}


CML::List<CML::PFrame> CML::Features::Relocalization::detectRelocalizationCandidates(PFrame frame) {
    FrameHashMap<int> lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    for (int i = 0; i < frame->numFeaturesGroup(); i++) if (frame->haveBoW(i)) {
            for (auto vit = frame->getBoW(i)->bowVec.begin(), vend = frame->getBoW(i)->bowVec.end(); vit != vend; vit++) {
                for (auto pKFi : mInvertedFile[vit->first]) {
                    if (lKFsSharingWords.count(pKFi) == 0) {
                        lKFsSharingWords.insert(Pair<PFrame, int>(pKFi, 0));
                    }
                    lKFsSharingWords[pKFi] += 1;
                }
            }
        }


    if(lKFsSharingWords.empty()) {
        return List<PFrame>();
    }

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(auto [pKFi, count] : lKFsSharingWords)
    {
        maxCommonWords = std::max(maxCommonWords, count);
    }
    int minCommonWords = maxCommonWords*0.8f;

    FrameHashMap<float> lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for (auto [pKFi, count] : lKFsSharingWords)
    {
        if(count > minCommonWords)
        {
            nscores++;
            float si = std::numeric_limits<float>::max();
            for (int i = 0; i < frame->numFeaturesGroup(); i++) if (frame->haveBoW(i)) for (int j = 0; j < pKFi->numFeaturesGroup(); j++) if (pKFi->haveBoW(j)) {
                float currentSi = mVocabulary.score(frame->getBoW(i)->bowVec, pKFi->getBoW(j)->bowVec);
                if (currentSi < si) si = currentSi;
            }
            //pKFi->mRelocScore=si;
            lScoreAndMatch.insert(Pair<PFrame, float>(pKFi,si));
        }
    }

    if(lScoreAndMatch.empty()) {
        return List<PFrame>();
    }

    LinkedList<Pair<float,PFrame>> lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(auto [pKFi, si] : lScoreAndMatch)
    {
        List<PFrame> vpNeighs = getMap().processIndirectCovisiblity(pKFi, 10, getMap().KEYFRAME);

        float bestScore = si;
        float accScore = bestScore;
        PFrame pBestKF = pKFi;
        for(auto pKF2 : vpNeighs)
        {
            if(lScoreAndMatch.count(pKF2) == 0) {
                continue;
            }

            float pKF2relocScore = lScoreAndMatch[pKF2];

            accScore += pKF2relocScore;
            if(pKF2relocScore > bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2relocScore;
            }

        }

        lAccScoreAndMatch.push_back(Pair<float, PFrame>(accScore,pBestKF));
        if(accScore>bestAccScore) {
            bestAccScore = accScore;
        }
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    FrameSet spAlreadyAddedKF;
    List<PFrame> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(auto [si, pKFi] : lAccScoreAndMatch)
    {
        if(si>minScoreToRetain)
        {
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

