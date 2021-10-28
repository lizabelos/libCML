#include "Hybrid.h"

bool Hybrid::poseEstimationDecision() {

    Vector6 currentVariance;
    currentVariance.head<3>() = mLastIndirectTrackingResult.covariance.tail<3>();
    currentVariance.tail<3>() = mLastPhotometricTrackingResidual.covariance.tail<3>();
    mTrackingDecisionCovariances.add(currentVariance);

    Vector6 v = mTrackingDecisionCovariances.accumulate(mTrackcondUncertaintyWindow.i());
    scalar_t vnorminv = 1.0 / v.norm();

    scalar_t indirectUncertainty = v.head<3>().norm() * vnorminv;
    scalar_t directUncertainty = v.tail<3>().norm() * vnorminv;

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

    if (mTrackcondUncertaintyWeight.f() > 0) {

        logger.important("ORB Uncertainty ( Pose Estimation Decision ) : " + std::to_string(indirectUncertainty));
        logger.important("DSO Uncertainty ( Pose Estimation Decision ) : " + std::to_string(directUncertainty));

        if (directUncertainty * mTrackcondUncertaintyWeight.f() < indirectUncertainty) {
            return true;
        }

        return false;

    }

    return false;

}

Hybrid::BaMode Hybrid::bundleAdjustmentDecision(bool needIndirectKF, bool needDirectKF) {

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

    if (mBaMinimumOrbPoint.i() > 0 && mLastNumTrackedPoints < mBaMinimumOrbPoint.i()) {
        return BADIRECT;
    }

    if (mBacondSaturatedRatio.f() > 0 && mLastPhotometricTrackingResidual.saturatedRatio() > mBacondSaturatedRatio.f()) {
        return BAINDIRECT;
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
        scalar_t vnorminv = 1.0 / v.norm();

        scalar_t indirectUncertainty = v.head<3>().norm() * vnorminv;
        scalar_t directUncertainty = v.tail<3>().norm() * vnorminv;

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


