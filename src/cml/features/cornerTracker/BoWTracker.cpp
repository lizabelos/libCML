#include "cml/features/cornerTracker/BoWTracker.h"

using namespace CML;

List<Pair<int, int>> CML::Features::BoWTracker::trackByProjection(const BoWFrameAndGroupAndDescriptor &A, const List<PPoint> &B, int th) {

    List<Pair<int, int>> result;

    auto cornersA = A.frame->getFeaturePoints(A.group);


    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP < B.size(); iMP++)
    {
        auto pMP = B[iMP];

        assertThrow(pMP->isGroup(getMap().INDIRECTGROUP), "This is not a indirect point");

        DistortedVector2d projection;
        int nPredictedLevel;
        scalar_t viewCos;
        if (!computeViewcosAndScale(pMP, A.frame, A.camera, projection, viewCos, nPredictedLevel)) {
            continue;
        }


        // The size of the window will depend on the viewing direction
        float r;
        if(viewCos>0.998) {
            r = 2.5;
        }
        else {
            r = 4.0;
        }

        if(bFactor) {
            r *= th;
        }

        List<NearestNeighbor> unfilteredNearestNeighbor = A.frame->processNearestNeighborsInRadius(A.group, projection, r * pMP->getReferenceCorner().processScaleFactorFromLevel(nPredictedLevel));
        int minLevel = nPredictedLevel-1, maxLevel = nPredictedLevel;

        List<NearestNeighbor> nearestNeighbor;
        for (auto nn : unfilteredNearestNeighbor) {
            auto corner = cornersA[nn.index];
            if(corner.level()<minLevel) {
                continue;
            }
            if(corner.level()>maxLevel) {
                continue;
            }
            nearestNeighbor.emplace_back(nn);
        }

        if(nearestNeighbor.empty())
            continue;

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(auto nn : nearestNeighbor)
        {
            auto corner = cornersA[nn.index];

            if (A.frame->getMapPoint(FeatureIndex(A.group, nn.index)).isNotNull()) {
                continue;
            }

            int dist = B[iMP]->getDescriptor<Binary256Descriptor>().distance(A.descriptors[nn.index]);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = corner.level();
                bestIdx=nn.index;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = corner.level();
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel == bestLevel2 && bestDist > mRatio * bestDist2)
                continue;

            result.emplace_back(bestIdx, iMP);
            nmatches++;
        }
    }

    return result;


}

List<Matching> CML::Features::BoWTracker::trackByBoW(const BoWFrameAndGroupAndDescriptor &A, const BoWFrameAndGroupAndDescriptor &B) {

    auto cornersTarget = A.frame->getFeaturePoints(A.group);
    auto bowTarget = A.frame->getBoW(A.group);

    auto cornersReference = B.frame->getFeaturePoints(B.group);
    auto bowReference = B.frame->getBoW(B.group);

    this->getTimer().start();

    List<int> vpMapPointMatches = List<int>(cornersTarget.size(),-1);

    List<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++) {
        rotHist[i].reserve(500);
    }
    const float factor = 1.0f / (float)HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    auto KFit = bowReference->featVec.begin();
    auto Fit = bowTarget->featVec.begin();
    auto KFend = bowReference->featVec.end();
    auto Fend = bowTarget->featVec.end();

    int nmatches = 0;

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const auto vIndicesKF = KFit->second;
            const auto vIndicesF = Fit->second;

            for (size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                OptPPoint pMP = B.frame->getMapPoint(FeatureIndex(B.group, realIdxKF));
                if (pMP.isNull()) {
                    continue;
                }

                if(!pMP->isGroup(getMap().MAPPED)) // todo : is this right ?
                    continue;

                const Binary256Descriptor &dKF = B.descriptors[realIdxKF];

                int bestDist1 = 256;
                int bestIdxF = -1;
                int bestDist2 = 256;

                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF] != -1) {
                        continue;
                    }

                    const Binary256Descriptor &dF = A.descriptors[realIdxF];

                    const int dist = dKF.distance(dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1 <= TH_LOW)
                {
                    if(static_cast<float>(bestDist1) < mRatio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF] = realIdxKF;

                        if(mCheckOrientation)
                        {
                            float rot = cornersReference[realIdxKF].angle() - cornersTarget[bestIdxF].angle();
                            if(rot<0.0) {
                                rot += 360.0f;
                            }
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = bowReference->featVec.lower_bound(Fit->first);
        }
        else
        {
            Fit = bowReference->featVec.lower_bound(KFit->first);
        }
    }


    if(mCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]] = -1;
                nmatches--;
            }
        }
    }

    List<Matching> matchings;
    for (size_t idTarget = 0; idTarget < cornersTarget.size(); idTarget++) {
        if (vpMapPointMatches[idTarget] != -1) {
            matchings.emplace_back(Matching(0, A.frame, B.frame, FeatureIndex(A.group, idTarget), FeatureIndex(B.group, vpMapPointMatches[idTarget])));

        }
    }

    {
        LockGuard lg(mLastMatchingMutex);
        mLastMatching = matchings;
    }

    mNumMatching->addValue(matchings.size());
    int mappedMatching = 0;
    for (auto matching : matchings) if (matching.getMapPoint().isNotNull()) mappedMatching++;
    mNumMatchingMapped->addValue(mappedMatching);


    this->getTimer().stop();

    return matchings;

}

List<Matching> CML::Features::BoWTracker::trackForInitialization(const BoWFrameAndGroupAndDescriptor &target, const BoWFrameAndGroupAndDescriptor &reference, int windowSize) {

    this->getTimer().start();

    auto cornersTarget = target.frame->getFeaturePoints(target.group);
    auto cornersReference = reference.frame->getFeaturePoints(reference.group);

    if (mInitializationLastReference != reference.frame) {
        mInitializationLastSeen = cornersReference;
        mInitializationLastReference = reference.frame;
    }

    List<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++) {
        rotHist[i].reserve(500);
    }
    const float factor = 1.0f / (float)HISTO_LENGTH;

    List<int> matchesTargToRefDist(cornersTarget.size(), INT_MAX);
    List<int> matchesTargToRef(cornersTarget.size(), -1);
    List<int> matchesRefToTarg(cornersReference.size(), -1);

    for (size_t iRef = 0; iRef < cornersReference.size(); iRef++) {

        auto kpRef = cornersReference[iRef];

        if (kpRef.level() > 0) {
            continue;
        }

        List<NearestNeighbor> unfilteredNearestNeighbor = target.frame->processNearestNeighborsInRadius(target.group, mInitializationLastSeen[iRef].point0(), windowSize);
        int minLevel = kpRef.level(), maxLevel = kpRef.level();

        List<NearestNeighbor> nearestNeighbor;
        for (auto nn : unfilteredNearestNeighbor) {
            auto corner = cornersTarget[nn.index];
            if(corner.level()<minLevel) {
                continue;
            }
            if(corner.level()>maxLevel) {
                continue;
            }
            nearestNeighbor.emplace_back(nn);
        }

        if(nearestNeighbor.empty()) {
            continue;
        }

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for (auto nn : nearestNeighbor) {

            int iTarget = nn.index;

            int dist = target.descriptors[iTarget].distance(reference.descriptors[iRef]);

            if(matchesTargToRefDist[iTarget] <= dist) {
                continue;
            }

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=iTarget;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }

        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mRatio)
            {
                if(matchesTargToRef[bestIdx2] >= 0)
                {
                    matchesRefToTarg[matchesTargToRef[bestIdx2]]=-1;
                }
                matchesRefToTarg[iRef]=bestIdx2;
                matchesTargToRef[bestIdx2]=iRef;
                matchesTargToRefDist[bestIdx2]=bestDist;

                if(mCheckOrientation)
                {
                    float rot = cornersReference[iRef].angle() - cornersTarget[bestIdx2].angle();
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(iRef);
                }
            }
        }
    }

    if(mCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(matchesRefToTarg[idx1] >= 0)
                {
                    matchesRefToTarg[idx1]=-1;
                }
            }
        }
    }

    List<Matching> matchings;
    for (size_t idTarget = 0; idTarget < cornersTarget.size(); idTarget++) {
        if (matchesTargToRef[idTarget] != -1) {
            matchings.emplace_back(Matching(0, target.frame, reference.frame, FeatureIndex(target.group, idTarget), FeatureIndex(reference.group, matchesTargToRef[idTarget])));
            mInitializationLastSeen[matchesTargToRef[idTarget]] = cornersTarget[idTarget];
        }
    }

    {
        LockGuard lg(mLastMatchingMutex);
        mLastMatching = matchings;
    }

    mNumMatching->addValue(matchings.size());
    int mappedMatching = 0;
    for (auto matching : matchings) if (matching.getMapPoint().isNotNull()) mappedMatching++;
    mNumMatchingMapped->addValue(mappedMatching);

    this->getTimer().stop();

    return matchings;


}

List<Matching> CML::Features::BoWTracker::trackForTriangulation(const BoWFrameAndGroupAndDescriptor &A, const BoWFrameAndGroupAndDescriptor &B) {
    auto cornersA = A.frame->getFeaturePoints(A.group);
    auto bowA = A.frame->getBoW(A.group);

    auto cornersB = B.frame->getFeaturePoints(B.group);
    auto bowB = B.frame->getBoW(B.group);

    this->getTimer().start();

    Matrix33 F12 = computeFundamental(A.camera, A.frame->getK(0), B.camera, B.frame->getK(0));

    const CML::Features::FeatureVector &vFeatVec1 = bowA->featVec;
    const CML::Features::FeatureVector &vFeatVec2 = bowB->featVec;

    Matrix33 K = A.frame->getK(0);
    scalar_t fx = K(0,0);
    scalar_t fy = K(1,1);
    scalar_t cx = K(0, 2);
    scalar_t cy = K(1, 2);

    //Compute epipole in second image
    // todo

    Matrix33 R1w = A.camera.getRotationMatrix();
    Vector3 t1w = A.camera.getTranslation();
    Vector3 Cw = -R1w * t1w;

    Matrix33 R2w = B.camera.getRotationMatrix();
    Vector3 t2w = B.camera.getTranslation();
    Vector3 C2 = R2w*Cw+t2w;

    const float invz = 1.0f/C2(2);
    const float ex =fx*C2(0)*invz+cx;
    const float ey =fy*C2(1)*invz+cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    List<bool> vbMatched2(cornersB.size(),false);
    List<int> vMatches12(cornersA.size(),-1);

    List<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    CML::Features::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    CML::Features::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    CML::Features::FeatureVector::const_iterator f1end = vFeatVec1.end();
    CML::Features::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                OptPPoint pMP1 = A.frame->getMapPoint(FeatureIndex(A.group, idx1));

                // If there is already a MapPoint skip
                if(pMP1.isNotNull()) {
                    continue;
                }

                const Corner &kp1 = cornersA[idx1];

                const Binary256Descriptor &d1 = A.descriptors[idx1];

                int bestDist = TH_LOW;
                int bestIdx2 = -1;

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];

                    OptPPoint pMP2 = B.frame->getMapPoint(FeatureIndex(B.group, idx2));

                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2.isNotNull())
                        continue;

                    const Binary256Descriptor &d2 = B.descriptors[idx2];

                    const int dist = d1.distance(d2);

                    if(dist>TH_LOW || dist>bestDist)
                        continue;

                    const Corner &kp2 = cornersB[idx2];


                    const float distex = ex-kp2.x();
                    const float distey = ey-kp2.y();
                    if(distex*distex+distey*distey<100*kp2.processScaleFactorFromLevel()) {
                        continue;
                    }


                    if(checkDistEpipolarLine(kp1,kp2,F12))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                if(bestIdx2>=0)
                {
                    const Corner &kp2 = cornersB[bestIdx2];
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;

                    if(mCheckOrientation)
                    {
                        float rot = kp1.angle() - kp2.angle();
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }


    List<Matching> matchings;
    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        Matching match(0, A.frame, B.frame, FeatureIndex(A.group, i), FeatureIndex(B.group, vMatches12[i]));
        assertThrow(match.getMapPoint().isNull(), "Track for triangulation matching must not have map point");
        matchings.emplace_back(match); // todo : check order
    }

    this->getTimer().stop();

    return matchings;
}

List<Matching> CML::Features::BoWTracker::trackByProjection(const BoWFrameAndGroupAndDescriptor &current, const BoWFrameAndGroup &last, int th) {
    int nmatches = 0;

    Matrix33 K = current.frame->getK(0);
    scalar_t fx = K(0,0);
    scalar_t fy = K(1,1);
    scalar_t cx = K(0, 2);
    scalar_t cy = K(1, 2);

    // Rotation Histogram (to check rotation consistency)
    List<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    Matrix33 Rcw = current.camera.getRotationMatrix();
    Vector3 tcw = current.camera.getTranslation();

    auto currentCorners = current.frame->getFeaturePoints(current.group);
    auto lastCorners = last.frame->getFeaturePoints(last.group);

    HashMap<int, int> matches;

    for(size_t i = 0; i < lastCorners.size(); i++)
    {
        OptPPoint pMP = last.frame->getMapPoint(FeatureIndex(last.group, i));

        if (pMP.isNull()) {
            continue;
        }

        // Project
        Vector3 x3Dw = pMP->getWorldCoordinate().absolute();
        Vector3 x3Dc = Rcw * x3Dw + tcw;

        const scalar_t xc = x3Dc(0);
        const scalar_t yc = x3Dc(1);
        const scalar_t invzc = 1.0 / x3Dc(2);

        if(invzc<0) {
            continue;
        }

        float u = fx*xc*invzc+cx;
        float v = fy*yc*invzc+cy;

        if (!current.frame->isInside(DistortedVector2d(u, v), 0, 0)) {
            continue;
        }

        int nLastOctave = lastCorners[i].level();

        // Search in a window. Size depends on scale
        float radius = (float)th * lastCorners[i].processScaleFactorFromLevel();



        List<NearestNeighbor> unfilteredNearestNeighbor = current.frame->processNearestNeighborsInRadius(current.group, Vector2(u,v), radius);
        int minLevel = nLastOctave - 1, maxLevel = nLastOctave + 1;

        List<NearestNeighbor> nearestNeighbor;
        for (auto nn : unfilteredNearestNeighbor) {
            auto corner = currentCorners[nn.index];
            if(corner.level()<minLevel) {
                continue;
            }
            if(corner.level()>maxLevel) {
                continue;
            }
            nearestNeighbor.emplace_back(nn);
        }

        if(nearestNeighbor.empty())
            continue;

        int bestDist = 256;
        int bestIdx2 = -1;

        for (size_t i2 = 0; i2 < currentCorners.size(); i2++)
        {
            OptPPoint pMP2 = current.frame->getMapPoint(FeatureIndex(current.group, i2));
            if (pMP2.isNotNull()) {
                continue;
            }
            //  if(CurrentFrame.mvpMapPoints[i2])
            //    if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
            //      continue; todo check this


            // todo : use point descriptor
            const int dist = current.descriptors[i2].distance(pMP->getDescriptor<Binary256Descriptor>());

            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx2=i2;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            matches[bestIdx2] = i;
            nmatches++;

            if(mCheckOrientation)
            {
                float rot = lastCorners[i].angle() - currentCorners[bestIdx2].angle();
                if(rot<0.0)
                    rot+=360.0f;
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(bestIdx2);
            }
        }

    }

    //Apply rotation consistency
    if(mCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    matches[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }
        }
    }

    List<Matching> matchings;
    for (auto [currentI, lastI] : matches) {
        if (currentI < 0 || lastI < 0) {
            continue;
        }
        matchings.emplace_back(
                Matching(0, current.frame, last.frame, FeatureIndex(current.group, currentI), FeatureIndex(last.group, lastI))
                );
    }

    return matchings;
}

Set<PPoint, Hasher> Features::BoWTracker::fuse(const Features::BoWFrameAndGroupAndDescriptor &A, const List<PPoint> &mapPoints) {

    auto cornersA = A.frame->getFeaturePoints(A.group);

    Set<PPoint, Hasher> fused;

    for (auto point : mapPoints) {

        if (!point->isGroup(getMap().MAPPED)) {
            continue;
        }

        if (A.frame->getIndex(point).hasValidValue()) {
            continue;
        }

        DistortedVector2d projection;
        int nPredictedLevel;
        scalar_t viewCos;
        if (!computeViewcosAndScale(point, A.frame, A.camera, projection, viewCos, nPredictedLevel)) {
            continue;
        }

        // Search in a radius
        const float radius = 3.0 * point->getReferenceCorner().processScaleFactorFromLevel(nPredictedLevel);

        List<NearestNeighbor> unfilteredNearestNeighbor = A.frame->processNearestNeighborsInRadius(A.group, projection, radius);
        int minLevel = nPredictedLevel-1, maxLevel = nPredictedLevel;

        List<NearestNeighbor> nearestNeighbor;
        for (auto nn : unfilteredNearestNeighbor) {
            auto corner = cornersA[nn.index];
            if(corner.level()<minLevel) {
                continue;
            }
            if(corner.level()>maxLevel) {
                continue;
            }
            nearestNeighbor.emplace_back(nn);
        }

        if(nearestNeighbor.empty())
            continue;

        int bestDist = 256;
        int bestIdx = -1;
        for(auto nn : nearestNeighbor)
        {
            auto corner = cornersA[nn.index];

            float e2 = (corner.point0() - projection).squaredNorm();

            if(e2 / pow(corner.processScaleFactorFromLevel(), 2) > 5.99)
                continue;

            const int dist = A.descriptors[nn.index].distance(point->getDescriptor<Binary256Descriptor>());

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = nn.index;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            OptPPoint pointInFrame = A.frame->getMapPoint(FeatureIndex(A.group, bestIdx));
            if(pointInFrame.isNotNull()) // If we found the point in A
            {
                // point is a point of a
                // pointInFrame is a fused point, so it's also a point of a
                // so their is a problem in merge map points
                if (getMap().mergeMapPoints(point, pointInFrame)) {
                    fused.insert(point);
                }
            }
            else
            {
                if (A.frame->setMapPoint(FeatureIndex(A.group, bestIdx), point)) {
                    fused.insert(point);
                }
            }
        }
    }

    return fused;

}
