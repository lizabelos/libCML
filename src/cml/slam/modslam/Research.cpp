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

    if (mLastFrame.isNotNull() && mTrackcondFlowThreshold.f() >= 0){

        scalar_t setting_maxShiftWeightT = 0.04 * (640 + 480);
        scalar_t setting_maxShiftWeightR = 0.0 * (640 + 480);
        scalar_t setting_maxShiftWeightRT = 0.02 * (640 + 480);
        scalar_t setting_maxAffineWeight = 2;

        Vector2 refToFh = mLastDirectKeyFrame->getExposure().to(mLastFrame->getExposure()).getParameters();

        bool flowTooBig = setting_maxShiftWeightT * CML::sqrt(mLastPhotometricTrackingResidual.flowVector[0]) /
                          (mLastFrame->getWidth(0) + mLastFrame->getHeight(0)) +
                          setting_maxShiftWeightR * CML::sqrt(mLastPhotometricTrackingResidual.flowVector[1]) /
                          (mLastFrame->getWidth(0) + mLastFrame->getHeight(0)) +
                          setting_maxShiftWeightRT * CML::sqrt(mLastPhotometricTrackingResidual.flowVector[2]) /
                          (mLastFrame->getWidth(0) + mLastFrame->getHeight(0)) +
                          setting_maxAffineWeight * abs(log(refToFh[0])) > mTrackcondFlowThreshold.f();

        if (flowTooBig) {
            return false;
        }
    }

    if (mTrackcondUncertaintyWeight.f() > 0) {

        CML_LOG_DEBUG("ORB Uncertainty ( Pose Estimation Decision ) : " + std::to_string(indirectUncertainty));
        CML_LOG_DEBUG("DSO Uncertainty ( Pose Estimation Decision ) : " + std::to_string(directUncertainty));

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

    Vector6 v = mBADecisionCovariances.accumulate(mBacondUncertaintyWindow.i());

    scalar_t indirectUncertainty = v.head<3>().norm();
    scalar_t directUncertainty = v.tail<3>().norm();

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

    if (mBacondTrackThresholdOrb.f() >= 0) {
        // dso is 0. orb is 1
        if (mBacondTrack.accumulate(10) > mBacondTrackThresholdOrb.f()) {
            return BAINDIRECT;
        }
    }

    if (mBacondTrackThresholdDso.f() >= 0) {
        // dso is 0. orb is 1
        if (mBacondTrack.accumulate(10) < mBacondTrackThresholdDso.f()) {
            return BADIRECT;
        }
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

        CML_LOG_IMPORTANT("ORB Score ( BA Decision ) : " + std::to_string(orbScore));
        CML_LOG_IMPORTANT("DSO Score ( BA Decision ) : " + std::to_string(dsoScore) + " -> " + std::to_string(weightedDsoScore));


        if (weightedDsoScore > orbScore) {
            return BADIRECT;
        } else {
            return BAINDIRECT;
        }

    }

    if (mBacondUncertaintyWeight.f() > 0) {

        if (!std::isfinite(indirectUncertainty)) {
            return BADIRECT;
        }

        if (!std::isfinite(directUncertainty)) {
            return BAINDIRECT;
        }

        CML_LOG_IMPORTANT("ORB Uncertainty ( BA Decision ) : " + std::to_string(indirectUncertainty));
        CML_LOG_IMPORTANT("DSO Uncertainty ( BA Decision ) : " + std::to_string(directUncertainty));

        if (directUncertainty * mBacondUncertaintyWeight.f() < indirectUncertainty) {
            return BADIRECT;
        } else {
            return BAINDIRECT;
        }

    }


    return NOBA;

}


