#include "cml/robust/RobustRaulmurInitializer.h"
#include "cml/maths/Utils.h"

bool CML::Robust::RobustRaulmurInitializer::track(CML::Map &map, CML::PFrame frameToTrack, CML::PFrame referenceFrame, CML::List<CML::Matching> matchings) {

    float minParallax = 1.0;
    int minTriangulated = 50;
    float sigma = 1.0;
    float sigma2 = sigma * sigma;

    if (matchings.size() < 30) {
        CML_LOG_WARN("Raulmur failed because of not enough matching");
        return false;
    }

    List<DistortedVector2d> frameToTrackDistorted, referenceDistorted;
    frameToTrackDistorted.resize(matchings.size());
    referenceDistorted.resize(matchings.size());

    List<UndistortedVector2d> frameToTrackUndistorted, referenceUndistorted;
    frameToTrackUndistorted.resize(matchings.size());
    referenceUndistorted.resize(matchings.size());

    for (size_t i = 0; i < matchings.size(); i++) {
        frameToTrackDistorted[i] = matchings[i].getFeaturePointA(frameToTrack).point(0);
        referenceDistorted[i] = matchings[i].getFeaturePointB(referenceFrame).point(0);
        frameToTrackUndistorted[i] = matchings[i].getUndistortedA(frameToTrack, 0);
        referenceUndistorted[i] = matchings[i].getUndistortedB(referenceFrame, 0);
    }

    Matrix33 normalizationMatrixA, normalizationMatrixB;

    List<NormalizedVector2d> normalizedA = normalize(frameToTrackDistorted, normalizationMatrixA);
    List<NormalizedVector2d> normalizedB = normalize(referenceDistorted, normalizationMatrixB);

    Matrix33 F, H;
    scalar_t scoreF = mRobustFundamental.compute(referenceDistorted, normalizedB, normalizationMatrixB, frameToTrackDistorted, normalizedA, normalizationMatrixA, F);
    scalar_t scoreH = mRobustHomography.compute(referenceDistorted, normalizedB, normalizationMatrixB, frameToTrackDistorted, normalizedA, normalizationMatrixA, H);

    CML_LOG_INFO("Robust Raulmur Fundamental Score : " + std::to_string(scoreF));
    CML_LOG_INFO("Robust Raulmur Homography Score : " + std::to_string(scoreH));

    List<Camera> hypothesisCamera;

    scalar_t ratioH = scoreH / (scoreH + scoreF);
    mRatioHomography->addValue(ratioH);

    List<Vector3> bestPoints3d;
    List<bool> bestInliers;
    Camera bestCamera;

    if (ratioH > 0.40) {
        hypothesisCamera = mRobustHomography.reconstruct(frameToTrack->getK(0));

        {
            LockGuard lg(mLastBestHypothesisMutex);
            mLastBestHypothesis = hypothesisCamera;
        }

        CML_LOG_INFO("Found Hypothesis : " + std::to_string(hypothesisCamera.size()));

        if (hypothesisCamera.empty()) {
            CML_LOG_WARN("Raulmur failed because of no hypothesis");
            return false;
        }

        int bestGood = 0;
        int secondBestGood = 0;
        float bestParallax;

        for (size_t i = 0; i < hypothesisCamera.size(); i++) {

            List<Vector3> points3d = mTriangulator.Triangulator::triangulate(hypothesisCamera[i], Camera(), frameToTrackUndistorted, referenceUndistorted, true); // todo : inverted A B ?
            List<bool> inliers;
            float parallax;
            int nGood = mCameraChecker.check(hypothesisCamera[i], Camera(), frameToTrackUndistorted, referenceUndistorted, points3d, 4.0 * sigma2, inliers, parallax);

            if (nGood > bestGood) {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestParallax = parallax;
                bestPoints3d = points3d;
                bestInliers = inliers;
                bestCamera = hypothesisCamera[i];
            } else if (nGood > secondBestGood) {
                secondBestGood = nGood;
            }

        }

        bestParallax = acos(bestParallax )*180.0/PI;

        if(secondBestGood > 0.75 * bestGood || bestParallax < minParallax || bestGood < minTriangulated || bestGood < 0.9 * (float)matchings.size())
        {
            return false;
        }
    } else {
        hypothesisCamera = mRobustFundamental.reconstruct(frameToTrack->getK(0));

        {
            LockGuard lg(mLastBestHypothesisMutex);
            mLastBestHypothesis = hypothesisCamera;
        }

        CML_LOG_INFO("Found Hypothesis : " + std::to_string(hypothesisCamera.size()));

        if (hypothesisCamera.empty()) {
            CML_LOG_WARN("Raulmur failed because of no hypothesis");
            return false;
        }

        List<Vector3> points3d[hypothesisCamera.size()];
        List<bool> inliers[hypothesisCamera.size()];
        float parallax[hypothesisCamera.size()];
        int nGoods[hypothesisCamera.size()];
        int maxGood = 0;
        int bestGood = 0;

        for (size_t i = 0; i < hypothesisCamera.size(); i++) {
            points3d[i] = mTriangulator.Triangulator::triangulate(hypothesisCamera[i], Camera(), frameToTrackUndistorted, referenceUndistorted, true);
            nGoods[i] = mCameraChecker.check(hypothesisCamera[i], Camera(), frameToTrackUndistorted, referenceUndistorted, points3d[i], 4.0 * sigma2, inliers[i], parallax[i]);
            if (nGoods[i] > maxGood) {
                maxGood = nGoods[i];
                bestGood = i;
            }
        }

        int nMinGood = max((int)(0.9*matchings.size()),minTriangulated);

        int nsimilar = 0;
        for (size_t i = 0; i < hypothesisCamera.size(); i++) {
            if (nGoods[i] > 0.7 * maxGood)
                nsimilar++;
        }

        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood<nMinGood || nsimilar>1)
        {
            return false;
        }

        float bestParallax = acos(parallax[bestGood] )*180.0/PI;
        if (bestParallax > minParallax) {
            bestPoints3d = points3d[bestGood];
            bestInliers = inliers[bestGood];
            bestCamera = hypothesisCamera[bestGood];
        } else {
            return false;
        }


    }


    List<double> allDepths;
    for (size_t i = 0; i < matchings.size(); i++) {
        if (bestInliers[i]) {
            allDepths.emplace_back(bestPoints3d[i].norm());
        }
    }
    double scaling = 1.0 / median(allDepths);

    frameToTrack->setCamera(bestCamera.withTranslation(bestCamera.getTranslation() * scaling));

    for (size_t i = 0; i < matchings.size(); i++) {
        if (bestInliers[i]) {
            PPoint newMapPoint = map.createMapPoint(referenceFrame, matchings[i].getIndexB(referenceFrame), INDIRECTTYPE);
            referenceFrame->setMapPoint(matchings[i].getIndexB(referenceFrame), newMapPoint);
            frameToTrack->setMapPoint(matchings[i].getIndexA(frameToTrack), newMapPoint);
            newMapPoint->setWorldCoordinate(WorldPoint::fromAbsolute(bestPoints3d[i] * scaling));
        }
    }

    referenceFrame->setGroup(getMap().INITFRAME, true);
    return true;

}
