#include "Hybrid.h"

bool Hybrid::poseEstimationDecision() {

    // true : should prefer dso
    // false : should prefer orb

    Vector6 currentVariance;
    currentVariance.head<3>() = mLastIndirectTrackingResult.covariance.tail<3>();
    currentVariance.tail<3>() = mLastPhotometricTrackingResidual.covariance.tail<3>();
    mTrackingDecisionCovariances.add(currentVariance);

    Vector6 v = mTrackingDecisionCovariances.accumulate(mTrackcondUncertaintyWindow.i());

    if (v.allFinite()) {
        v.normalize();
    }

    scalar_t indirectUncertainty = v.head<3>().norm();
    scalar_t directUncertainty = v.tail<3>().norm();

    if (!mLastPhotometricTrackingResidual.isCorrect) {
       // mStatTrackORBVar->addValue(indirectUncertainty);
       // mStatTrackDSOVar->addValue(indirectUncertainty * 2);
        return false;
    }

    mStatTrackORBVar->addValue(indirectUncertainty);
    mStatTrackDSOVar->addValue(directUncertainty);

    if (mTrackcondForce.i() == 1) {
        return false;
    }

    if (mTrackcondForce.i() == 2) {
        return true;
    }

    if (mTrackcondForce.i() == 3) {
        return !mShouldPreferDso;
    }

    if (mTrackcondUncertaintyWeightOrb.f() > 0) {

        if (!std::isfinite(indirectUncertainty)) {
            return true;
        }

        if (!std::isfinite(directUncertainty)) {
            return false;
        }

        if (indirectUncertainty * mTrackcondUncertaintyWeightOrb.f() < directUncertainty) {
            return false;
        }

    }

    if (mTrackcondUncertaintyWeightDso.f() > 0) {

        if (!std::isfinite(indirectUncertainty)) {
            return true;
        }

        if (!std::isfinite(directUncertainty)) {
            return false;
        }

        if (directUncertainty * mTrackcondUncertaintyWeightDso.f() < indirectUncertainty) {
            return true;
        }

    }

    if (mTrackingMinimumOrbPoint.i() >= 0 && mLastNumTrackedPoints < mTrackingMinimumOrbPoint.i()) {
        return true;
    }

    if (mTrackcondUncertaintyWeight.f() > 0) {

        logger.debug("ORB Uncertainty ( Pose Estimation Decision ) : " + std::to_string(indirectUncertainty));
        logger.debug("DSO Uncertainty ( Pose Estimation Decision ) : " + std::to_string(directUncertainty));

        if (!std::isfinite(indirectUncertainty)) {
            return true;
        }

        if (!std::isfinite(directUncertainty)) {
            return false;
        }

        if (directUncertainty * mTrackcondUncertaintyWeight.f() < indirectUncertainty) {
            return true;
        }

        return false;

    }

    return false;

}

Hybrid::BaMode Hybrid::bundleAdjustmentDecision(bool needIndirectKF, bool needDirectKF) {

    if (needIndirectKF && mBaOrbRepeat.i() >= 0) {
        if (getMap().getLastGroupFrame(INDIRECTKEYFRAME)->getId() + mBaOrbRepeat.i() > getMap().getLastFrame()->getId()) {
            return BAINDIRECT;
        }
    }

    Vector6 currentVariance;
    currentVariance.head<3>() = mLastIndirectTrackingResult.covariance.tail<3>();
    currentVariance.tail<3>() = mLastPhotometricTrackingResidual.covariance.tail<3>();
    mBADecisionCovariances.add(currentVariance);

    scalar_t currentOrbScore = mLastNumTrackedPoints;
    scalar_t currentDsoScore = 0;
    if (mLastPhotometricTrackingResidual.numRobust.size() > 0) {
        currentDsoScore = mLastPhotometricTrackingResidual.numRobust[0];
    }
    mBADecisionScores.add(Vector2(currentOrbScore, currentDsoScore));

    Vector2 scores = mBADecisionScores.accumulate(mScoreWindow.i());
    scalar_t orbScore = scores(0);
    scalar_t dsoScore = scores(1);
    scalar_t weightedDsoScore = dsoScore * mScoreWeight.f();

    mStatBAORBNum->addValue(orbScore);
    mStatBADSONum->addValue(dsoScore);

    if (mBacondForce.i() == 1) {
        return BAINDIRECT;
    }

    if (mBacondForce.i() == 2) {
        return BADIRECT;
    }

    if (mBacondForce.i() == 3) {
        if (mBaMode == BAINDIRECT) return BADIRECT;
        else return BAINDIRECT;
    }

    if (mBaMinimumOrbPoint.i() >= 0 && mLastNumTrackedPoints < mBaMinimumOrbPoint.i()) {
        return BADIRECT;
    }

    if (mBacondSaturatedRatioDir.b() == false) {
        if (mBacondSaturatedRatio.f() > 0 && mLastPhotometricTrackingResidual.saturatedRatio() < mBacondSaturatedRatio.f()) {
            return BADIRECT;
        }
    } else {
        if (mBacondSaturatedRatio.f() > 0 && mLastPhotometricTrackingResidual.saturatedRatio() > mBacondSaturatedRatio.f()) {
            return BAINDIRECT;
        }
    }

    if (mScoreWeight.f() >= 0) {

        logger.important("ORB Score ( BA Decision ) : " + std::to_string(orbScore));
        logger.important("DSO Score ( BA Decision ) : " + std::to_string(dsoScore) + " -> " + std::to_string(weightedDsoScore));


        if (weightedDsoScore > orbScore) {
            return BADIRECT;
        } else {
            return BAINDIRECT;
        }

    }

    if (mBacondUncertaintyWeight.f() > 0) {

        Vector6 v = mBADecisionCovariances.accumulate(mBacondUncertaintyWindow.i());

        scalar_t indirectUncertainty = v.head<3>().norm();
        scalar_t directUncertainty = v.tail<3>().norm();

        if (!std::isfinite(indirectUncertainty)) {
            return BADIRECT;
        }

        if (!std::isfinite(directUncertainty)) {
            return BAINDIRECT;
        }

        logger.important("ORB Uncertainty ( BA Decision ) : " + std::to_string(indirectUncertainty));
        logger.important("DSO Uncertainty ( BA Decision ) : " + std::to_string(directUncertainty));

        if (directUncertainty * mBacondUncertaintyWeight.f() < indirectUncertainty) {
            return BADIRECT;
        } else {
            return BAINDIRECT;
        }

    }


    return NOBA;

}


