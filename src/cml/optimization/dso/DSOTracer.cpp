#include "cml/optimization/dso/DSOTracer.h"
#include "cml/utils/KDTree.h"
#include "cml/utils/DistanceMap.h"

CML::Optimization::DSOTracer::DSOTracer(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {
    mPrivateDataInstance = getMap().getMapPointsPrivataDataContext().createInstance();
}

CML::Optimization::DSOTracer::~DSOTracer() {

}

void CML::Optimization::DSOTracer::traceNewCoarse(PFrame frameToTrace, int frameGroup) {

    int trace_total=0, trace_good=0, trace_oob=0, trace_out=0, trace_skip=0, trace_badcondition=0, trace_uninitialized=0;

    Set<PPoint, Hasher> toRemove;

    for (auto point : getMap().getGroupMapPoints(IMMATUREPOINT)) {

        if (!point->getReferenceFrame()->isGroup(frameGroup)) {
            assertDeterministic("Removing " + std::to_string(point->getId()) + " reference frame does not belong to the group");
            toRemove.insert(point);
            continue;
        }

        auto ph = getPrivateData(point);

        if (point->getReferenceFrame()->getMapPoint(ph->referenceIndex).isNotNull()) {
            assertDeterministic("Removing " + std::to_string(point->getId()));
            toRemove.insert(point);
            continue;
        }

        trace(frameToTrace, point);

        assertDeterministic("Point " + std::to_string(point->getId()) + " --> " + toString(ph->lastTraceStatus));

        if(ph->lastTraceStatus == IPS_GOOD) trace_good++;
        if(ph->lastTraceStatus == IPS_BADCONDITION) trace_badcondition++;
        if(ph->lastTraceStatus == IPS_OOB) trace_oob++;
        if(ph->lastTraceStatus == IPS_OUTLIER) trace_out++;
        if(ph->lastTraceStatus == IPS_SKIPPED) trace_skip++;
        if(ph->lastTraceStatus == IPS_UNINITIALIZED) trace_uninitialized++;

        trace_total++;

    }

    for (auto point : toRemove) {
        getMap().removeMapPoint(point, true);
    }

}

CML::Set<CML::PPoint, CML::Hasher> CML::Optimization::DSOTracer::activatePoints(int frameGroup, int pointGroup) {

    auto points = getMap().getGroupMapPoints(pointGroup);
    
    if((int)points.size() < mSettingsDesiredPointDensity.i() * 0.66)
        mCurrentMinimumDistance -= 0.8;
    if((int)points.size() < mSettingsDesiredPointDensity.i() * 0.8)
        mCurrentMinimumDistance -= 0.5;
    else if((int)points.size() < mSettingsDesiredPointDensity.i() * 0.9)
        mCurrentMinimumDistance -= 0.2;
    else if((int)points.size() < mSettingsDesiredPointDensity.i())
        mCurrentMinimumDistance -= 0.1;

    if((int)points.size() > mSettingsDesiredPointDensity.i() * 1.5)
        mCurrentMinimumDistance += 0.8;
    if((int)points.size() > mSettingsDesiredPointDensity.i() * 1.3)
        mCurrentMinimumDistance += 0.5;
    if((int)points.size() > mSettingsDesiredPointDensity.i() * 1.15)
        mCurrentMinimumDistance += 0.2;
    if((int)points.size() > mSettingsDesiredPointDensity.i())
        mCurrentMinimumDistance += 0.1;

    if(mCurrentMinimumDistance < 0) {
        mCurrentMinimumDistance = 0;
        logger.warn("We need urgently some new points !!");
    }
    if(mCurrentMinimumDistance > 4) mCurrentMinimumDistance = 4;

    mStatisticMinimumDistance->addValue(mCurrentMinimumDistance);

    OptPFrame lastFrame = getMap().getLastGroupFrame(frameGroup);

    List<DistortedVector2d> projectedActivePoints;
    for (PPoint point : points) {
        UndistortedVector2d p = point->getWorldCoordinate().project(lastFrame->getCamera());
        projectedActivePoints.emplace_back(lastFrame->distort(p, 0));
    }

    float maxType = 4;
    DistanceMap distanceMap(lastFrame->getWidth(0), lastFrame->getHeight(0), mCurrentMinimumDistance * maxType);
    distanceMap.addPoints(projectedActivePoints);



    //coarseTracker->debugPlotDistMap("distMap");

    List<PPoint> toOptimize;
    List<PPoint> toRemove;

    int numDeletedBecauseOutlier = 0;
    int numDeletedBecauseOOB = 0;

    int numSkippedBecauseStatus = 0;
    int numSkippedBecausePixelInterval = 0;
    int numSkippedBecauseQuality = 0;
    int numSkippedBecauseDepth = 0;

    for (auto point : getMap().getGroupMapPoints(IMMATUREPOINT)) {
        auto pointData = getPrivateData(point);

        if (point->getReferenceFrame() == lastFrame) {
            continue;
        }

        if (!point->getReferenceFrame()->isGroup(frameGroup)) {
            toRemove.emplace_back(point);
            continue;
        }

        if(!std::isfinite(pointData->iDepthMax) || pointData->lastTraceStatus == IPS_OUTLIER)
        {
            toRemove.emplace_back(point);
            numDeletedBecauseOutlier++;
            continue;
        }

        bool canActivateWrtLastTraceStatus = (pointData->lastTraceStatus == IPS_GOOD
                                              || pointData->lastTraceStatus == IPS_SKIPPED
                                              || pointData->lastTraceStatus == IPS_BADCONDITION
                                              || pointData->lastTraceStatus == IPS_OOB );

        bool canActivateWrtLastTracePixelInterval = pointData->lastTracePixelInterval < 8;

        bool canActivateWrtQuality = pointData->quality > mSettingsMinTraceQuality.f();

        bool canActivateWrtDepth = (pointData->iDepthMax+pointData->iDepthMin) > 0;

        // can activate only if this is true.
        bool canActivate = canActivateWrtLastTraceStatus
                           && canActivateWrtLastTracePixelInterval
                           && canActivateWrtQuality
                           && canActivateWrtDepth;

        if (!canActivateWrtLastTraceStatus) {
            numSkippedBecauseStatus++;
        }

        if (!canActivateWrtLastTracePixelInterval) {
            numSkippedBecausePixelInterval++;
        }

        if (!canActivateWrtQuality) {
            numSkippedBecauseQuality++;
        }

        if (!canActivateWrtDepth) {
            numSkippedBecauseDepth++;
        }


        // if I cannot activate the point, skip it. Maybe also delete it.
        if(!canActivate)
        {
            // if point will be out afterwards, delete it instead.
            if(pointData->lastTraceStatus == IPS_OOB)
            {
                toRemove.emplace_back(point);
                numDeletedBecauseOOB++;
                continue;
            }
            continue;
        }


        // see if we need to activate point due to distance map.
        scalar_t idepth = (pointData->iDepthMin + pointData->iDepthMax) / 2.0;
        UndistortedVector2d undistortedPoint = point->getWorldCoordinateIf(idepth, {0, 0}).project(lastFrame->getCamera());
        DistortedVector2d p = lastFrame->distort(undistortedPoint, 0);

        if (lastFrame->isInside(p, 0, 0)) {
            scalar_t dist = distanceMap.get(p) + (p.x() - floor(p.x()));
            if(dist >= mCurrentMinimumDistance * pointData->my_type)
            {
                distanceMap.addPoint(p);
                toOptimize.push_back(point);
            }
        }
        else
        {
            toRemove.emplace_back(point);
            continue;
        }

    }

    float numImmaturePoints = getMap().getGroupMapPoints(IMMATUREPOINT).size();

    mStatisticNumDeleteBecauseOutlier->addValue((float)numDeletedBecauseOutlier / numImmaturePoints);
    mStatisticNumDeleteBecauseOOB->addValue((float)numDeletedBecauseOOB / numImmaturePoints);
    mStatisticNumSkippedBecauseStatus->addValue((float)numSkippedBecauseStatus / numImmaturePoints);
    mStatisticNumSkippedBecausePixelInterval->addValue((float)numSkippedBecausePixelInterval / numImmaturePoints);
    mStatisticNumSkippedBecauseQuality->addValue((float)numSkippedBecauseQuality / numImmaturePoints);
    mStatisticNumSkippedBecauseDepth->addValue((float)numSkippedBecauseDepth / numImmaturePoints);

    List<int> optimizationResult;
    optimizationResult.resize(toOptimize.size());

    #if CML_USE_OPENMP
    #pragma omp for schedule(static)
    #endif
    for (size_t i = 0; i < toOptimize.size(); i++) {
        optimizationResult[i] = optimizeImmaturePoint(toOptimize[i], 1, frameGroup);
    }

    List<Vector3f> lastTraced;
    Set<PPoint, Hasher> mappedPoints;

    int numMapped = 0, numNonMapped = 0, numDrop = 0;
    for(size_t k = 0; k < toOptimize.size(); k++)
    {
        int result = optimizationResult[k];
        auto pointData = getPrivateData(toOptimize[k]);

        if(result == 1)
        {
            if (toOptimize[k]->getReferenceFrame()->setMapPoint(getPrivateData(toOptimize[k])->referenceIndex, toOptimize[k])) {
                numMapped++;
                lastTraced.emplace_back(toOptimize[k]->getWorldCoordinate().absolute().cast<float>());
                mappedPoints.insert(toOptimize[k]);
                freePrivateData(toOptimize[k], "DSOTracer::activatePoints");
            } else {
                freePrivateData(toOptimize[k], "DSOTracer::activatePoints");
                getMap().removeMapPoint(toOptimize[k]);
            }
        }
        else if(result == -1 || pointData->lastTraceStatus==IPS_OOB)
        {
            toRemove.emplace_back(toOptimize[k]);
            numDrop++;
        }
        else
        {
            numNonMapped++;
            // Do nothing...
        }
    }

    mStatisticNumProposition->addValue(numImmaturePoints);
    mStatisticNumMapped->addValue(numMapped);

    mStatisticNumNonMapped->addValue(numNonMapped);
    mStatisticNumDropped->addValue(numDrop);

    for (auto point : toRemove) {
        getMap().removeMapPoint(point, true);
    }

    LockGuard lg(mLastTracedMutex);
    mLastLastTraced = mLastTraced;
    mLastTraced = lastTraced;

    return mappedPoints;

}

int CML::Optimization::DSOTracer::optimizeImmaturePoint(PPoint point, int minObs, int frameGroup) {

    if (!point->getReferenceFrame()->isGroup(frameGroup)) {
        logger.error("Immature points not cleaned up correctly");
        getMap().removeMapPoint(point, true);
        return -1;
    }

    auto pointData = getPrivateData(point);

    auto frames = getMap().getGroupFrames(frameGroup);

    ImmaturePointTemporaryResidual residuals[frames.size()];

    int nres = 0;
    for(auto frame : frames)
    {
        if (frame != point->getReferenceFrame()) {
            residuals[nres].state_NewEnergy = residuals[nres].state_energy = 0;
            residuals[nres].state_NewState = DSOResidualState::DSORES_OUTLIER;
            residuals[nres].state_state = DSOResidualState::DSORES_IN;
            residuals[nres].target = frame;
            nres++;
        }
    }

    if (nres != ((int)frames.size())-1) {
        logger.error("Immature points not cleaned up correctly (number of res not corresponding)");
        getMap().removeMapPoint(point, true);
        return -1;
    }

    float lastEnergy = 0;
    float lastHdd=0;
    float lastbd=0;
    float currentIdepth=(pointData->iDepthMax + pointData->iDepthMin) * 0.5f;


    for(int i=0;i<nres;i++)
    {
        lastEnergy += linearizeResidual(point, 1000, &residuals[i],lastHdd, lastbd, currentIdepth);
        residuals[i].state_state = residuals[i].state_NewState;
        residuals[i].state_energy = residuals[i].state_NewEnergy;
    }

    if(!std::isfinite(lastEnergy) || lastHdd <  mMinIDepthHAct.f())
    {
        return 0;
    }

    float lambda = 0.1;
    for(int iteration = 0; iteration < mGNItsOnPointActation.i(); iteration++)
    {
        float H = lastHdd;
        H *= 1+lambda;
        float step = (1.0/H) * lastbd;
        float newIdepth = currentIdepth - step;

        float newHdd=0; float newbd=0; float newEnergy=0;
        for(int i=0;i<nres;i++) {
            newEnergy += linearizeResidual(point, 1, residuals+i,newHdd, newbd, newIdepth);
        }

        if(!std::isfinite(lastEnergy) || newHdd < mMinIDepthHAct.f())
        {
            return 0;
        }


        if(newEnergy < lastEnergy)
        {
            currentIdepth = newIdepth;
            lastHdd = newHdd;
            lastbd = newbd;
            lastEnergy = newEnergy;
            for(int i=0;i<nres;i++)
            {
                residuals[i].state_state = residuals[i].state_NewState;
                residuals[i].state_energy = residuals[i].state_NewEnergy;
            }

            lambda *= 0.5;
        }
        else
        {
            lambda *= 5;
        }

        if(fabsf(step) < 0.0001*currentIdepth)
            break;
    }

    if(!std::isfinite(currentIdepth))
    {
        return -1;
    }

    if (currentIdepth <= 0) {
        logger.error("Negative inverse depth after initialization");
        return -1;
    }


    int numGoodRes = 0, numOutliers = 0;
    for(int i=0;i<nres;i++) {
        if (residuals[i].state_state == DSOResidualState::DSORES_IN) numGoodRes++;
        if (residuals[i].state_state == DSOResidualState::DSORES_OUTLIER) numOutliers++;
    }

    if(numGoodRes < minObs)
    {
        return -1;
    }

  /*  if (numOutliers > 0) { // TODO : Added by me
        return -1;
    } */

    if(!std::isfinite(pointData->energyTH)) {
        return -1;
    }

    point->setReferenceInverseDepth(currentIdepth);
    point->setGroup(IMMATUREPOINT, false);

    for(int i=0;i<nres;i++) {
        if (residuals[i].state_state == DSOResidualState::DSORES_IN) {
            residuals[i].target->addDirectApparitions(point);
        }
    }

    return 1;
}

double CML::Optimization::DSOTracer::linearizeResidual(PPoint point, float outlierTHSlack, ImmaturePointTemporaryResidual* tmpRes, float &Hdd, float &bd, float idepth) {
    if(tmpRes->state_state == DSORES_OOB)
    {
        tmpRes->state_NewState = DSORES_OOB;
        return tmpRes->state_energy;
    }

    auto frame = tmpRes->target;
    auto pointData = getPrivateData(point);
    // check OOB due to scale angle change.

    float energyLeft=0;

    Camera hostToTarget = point->getReferenceFrame()->getCamera().to(frame->getCamera());
    // ExposureTransition exposureTransition = point->getReferenceFrame()->getExposure().to(frame->getExposure()); // TODO : Check this
    ExposureTransition exposureTransition = point->getReferenceFrame()->getExposure().to(frame->getExposure());
    // ExposureTransition exposureTransition = getPrivateData(point->getReferenceFrame())->aff_g2l().to(getPrivateData(frame)->aff_g2l());


    for (size_t idx = 0; idx < mPattern.size(); idx++)
    {

        DistortedVector2d refcorner_Distorted(point->getReferenceCorner().point0() + mPattern[idx]);
        UndistortedVector2d refcorner = point->getReferenceFrame()->undistort(refcorner_Distorted, 0);
        Vector3 projectedcurp = hostToTarget.getRotationMatrix() * refcorner.homogeneous() + hostToTarget.getTranslation() * idepth;
        UndistortedVector2d undistortedProjection(projectedcurp.hnormalized());
        DistortedVector2d projection = frame->distort(undistortedProjection, 0);
        scalar_t drescale = 1.0 / projectedcurp[2];

        if(!frame->isInside(projection, 0, 1) || drescale <= 0)
        {
            tmpRes->state_NewState = DSORES_OOB;
            return tmpRes->state_energy;
        }

        //scalar_t hitColor = frame->getCaptureFrame().getGrayImage(0).interpolate(Vector2(projection.x(), projection.y()));
        Vector3f gradientVector = frame->getCaptureFrame().getDerivativeImage(0).interpolate(Vector2f(projection.x(), projection.y()));
        scalar_t hitColor = gradientVector[0];

        // scalar_t groundtruth = point->getGrayPatch(mPattern[idx].x(), mPattern[idx].y(), 0);
        auto groundtruthAndGradient = point->getDerivativePatch(mPattern[idx].x(), mPattern[idx].y(), 0);

        scalar_t groundtruth = exposureTransition(groundtruthAndGradient(0));

        scalar_t residual = hitColor - groundtruth;

        scalar_t hw = fabs(residual) < mSettingHuberTH.f() ? 1 : mSettingHuberTH.f() / fabs(residual);

        scalar_t weight = sqrt(mSettingOutlierTHSumComponent.f() / (mSettingOutlierTHSumComponent.f() + groundtruthAndGradient.tail<2>().squaredNorm()));

        energyLeft += weight * weight * hw * residual * residual * (2 - hw);

        // depth derivatives.
        Matrix33 K = point->getReferenceFrame()->getK(0);

        scalar_t dxInterp = gradientVector[1] * K(0, 0);
        scalar_t dyInterp = gradientVector[2] * K(1, 1);

        Vector3 t = hostToTarget.getTranslation();

        scalar_t d_idepth = (dxInterp * drescale * (t[0] - t[2] * undistortedProjection.x()) + dyInterp * drescale * (t[1] - t[2] * undistortedProjection.y()));

        hw *= weight * weight;

        Hdd += ( hw * d_idepth) * d_idepth;
        bd += ( hw * residual) * d_idepth;
    }


    if(energyLeft > pointData->energyTH * outlierTHSlack)
    {
        energyLeft = pointData->energyTH * outlierTHSlack;
        tmpRes->state_NewState = DSORES_OUTLIER;
    }
    else
    {
        tmpRes->state_NewState = DSORES_IN;
    }

    tmpRes->state_NewEnergy = energyLeft;
    return energyLeft;
}

void CML::Optimization::DSOTracer::makeNewTraces(PFrame frame) {
    List<Corner> corners;
    List<float> types;
    if (mPixelSelector == nullptr) {
        mPixelSelector = new Features::PixelSelector(this, frame->getWidth(0), frame->getHeight(0));
    }
    mPixelSelector->compute(frame->getCaptureFrame(), corners, types, mDesiredImmatureDensity.i());

    int groupId = frame->addFeaturePoints(corners);

    for (size_t i = 0; i < frame->getFeaturePoints(groupId).size(); i++) {

        // auto corner = corners[i];
        auto type = types[i];

        auto point = getMap().createMapPoint(frame, FeatureIndex(groupId, i), DIRECT);
        auto pointData = getPrivateData(point);

        pointData->referenceIndex = FeatureIndex(groupId, i);
        pointData->gradH.setZero();
        pointData->weights.resize(mPattern.size());

        DistortedVector2d distortedReferenceCorner = point->getReferenceCorner().point(0);
        for (size_t patternId = 0; patternId < mPattern.size(); patternId++) {
            const Vector2 &shift = mPattern[patternId];
            Vector2 grad = point->getReferenceFrame()->getCaptureFrame().getDerivativeImage(0).interpolate(Vector2f(distortedReferenceCorner.x() + shift.x(), distortedReferenceCorner.y() + shift.y())).tail<2>().cast<scalar_t>();
            pointData->gradH += grad * grad.transpose();
            pointData->weights[patternId] = sqrt(mSettingOutlierTHSumComponent.f() / (mSettingOutlierTHSumComponent.f() + grad.squaredNorm()));
        }
        pointData->energyTH = mPattern.size() * mSettingOutlierTH.f();
        pointData->my_type = type;
        // pointData->energyTH *= setting_overallEnergyTHWeight*setting_overallEnergyTHWeight;

        if (!std::isfinite(pointData->energyTH)) {
            getMap().removeMapPoint(point, true);
            continue;
        }

        point->setGroup(IMMATUREPOINT, true);


    }

}

void CML::Optimization::DSOTracer::makeNewTracesFrom(PFrame frame, int groupId) {

    int s = frame->getFeaturePoints(groupId).size();

    for (int i = 0; i < s; i++) {

        if (frame->getMapPoint(FeatureIndex(groupId, i)).isNotNull()) {
            continue;
        }

        //auto corner = frame->getFeaturePoint(FeatureIndex(groupId, i)); // todo : type from scale factor ?
        auto type = 1;

        auto point = getMap().createMapPoint(frame, FeatureIndex(groupId, i), DIRECT);
        auto pointData = getPrivateData(point);

        pointData->referenceIndex = FeatureIndex(groupId, i);

        pointData->gradH.setZero();
        pointData->weights.resize(mPattern.size());

        DistortedVector2d distortedReferenceCorner = point->getReferenceCorner().point(0);
        for (size_t patternId = 0; patternId < mPattern.size(); patternId++) {
            const Vector2 &shift = mPattern[patternId];
            Vector2 grad = point->getReferenceFrame()->getCaptureFrame().getDerivativeImage(0).interpolate(Vector2f(distortedReferenceCorner.x() + shift.x(), distortedReferenceCorner.y() + shift.y())).tail<2>().cast<scalar_t>();
            pointData->gradH += grad * grad.transpose();
            pointData->weights[patternId] = sqrt(mSettingOutlierTHSumComponent.f() / (mSettingOutlierTHSumComponent.f() + grad.squaredNorm()));
        }
        pointData->energyTH = mPattern.size() * mSettingOutlierTH.f();
        pointData->my_type = type;
        // pointData->energyTH *= setting_overallEnergyTHWeight*setting_overallEnergyTHWeight;

        if (!std::isfinite(pointData->energyTH)) {
            getMap().removeMapPoint(point, true);
            continue;
        }

        point->setGroup(IMMATUREPOINT, true);


    }

}

CML::Optimization::DSOTracerStatus CML::Optimization::DSOTracer::trace(PFrame frame, PPoint mapPoint) {

    //int level = frame->getCaptureFrame().getNearestLevel(300, 300);
    int level = 0;

    // ExposureTransition exposureTransition = getPrivateData(mapPoint->getReferenceFrame())->aff_g2l().to(getPrivateData(frame)->aff_g2l());
    ExposureTransition exposureTransition = mapPoint->getReferenceFrame()->getExposure().to(frame->getExposure());

    auto self = getPrivateData(mapPoint);
    PFrame referenceFrame = mapPoint->getReferenceFrame();
    DistortedVector2d distortedReferenceCorner = mapPoint->getReferenceCorner().point(level);

    if (frame == referenceFrame) {
        return self->lastTraceStatus;
    }

    if (self->lastTraceStatus == IPS_OOB) {
        assertDeterministic("OOB because last OOB");
        return IPS_OOB;
    }

    Matrix33 hostToFrame_KRKi = frame->getK(level) * referenceFrame->getCamera().to(frame->getCamera()).getRotationMatrix() * frame->getK(level).inverse(); // TODO : Check this. Maybe K is inverted ????
    Vector3 hostToFrame_Kt = frame->getK(level) * referenceFrame->getCamera().to(frame->getCamera()).getTranslation();
    Vector3 pr = hostToFrame_KRKi * Vector3(distortedReferenceCorner.x(), distortedReferenceCorner.y(), 1);

    //UndistortedVector2d undistortedReferenceCorner = referenceFrame->undistort(distortedReferenceCorner, level);
    scalar_t maxPixSearch = (scalar_t)(frame->getWidth(level) + frame->getHeight(level)) * mSettingMaxPixSearch.f();

    /// Check if the projection of the minimum point is inside the frame
    Vector3 ptpMin = pr + hostToFrame_Kt * self->iDepthMin;
    DistortedVector2d distortedProjectedPtpMin(ptpMin.hnormalized());
    // WorldPoint ptpMin = WorldPoint::fromInverseDepth(self->iDepthMin, undistortedReferenceCorner, referenceFrame->getCamera());
    // UndistortedVector2d undistortedProjectedPtpMin = ptpMin.project(frame->getCamera());
    // DistortedVector2d distortedProjectedPtpMin = frame->distort(undistortedProjectedPtpMin, level);

    if (!frame->isInside(distortedProjectedPtpMin, level, 4)) {
        self->lastTraceUV = Vector2(-1, -1);
        self->lastTracePixelInterval = 0;
        self->lastTraceStatus = IPS_OOB;
        assertDeterministic("OOB because not inside frame");
        return IPS_OOB;
    }

    /// Check if the projection of the maximum point is inside the frame
    //UndistortedVector2d undistortedProjectedPtpMax;
    DistortedVector2d distortedProjectedPtpMax;
    scalar_t pixelInterval;

    if (std::isfinite(self->iDepthMax)) {

        Vector3 ptpMax = pr + hostToFrame_Kt * self->iDepthMax;
        distortedProjectedPtpMax = DistortedVector2d(ptpMax.hnormalized());

        if (!frame->isInside(distortedProjectedPtpMax, level, 5)) {
            self->lastTraceUV = Vector2(-1, -1);
            self->lastTracePixelInterval = 0;
            self->lastTraceStatus = IPS_OOB;
            assertDeterministic("OOB because max not in frame (finite)");
            return IPS_OOB;
        }

        // If the interval is too low, skip
        pixelInterval = (distortedProjectedPtpMax - distortedProjectedPtpMin).norm();
        if (pixelInterval < mSettingMaxSlackInterval.f()) {
            self->lastTraceUV = (distortedProjectedPtpMax + distortedProjectedPtpMin) / 2.0;
            self->lastTracePixelInterval = pixelInterval;
            self->lastTraceStatus = IPS_SKIPPED;
            return IPS_SKIPPED;
        }

    } else {

        pixelInterval = maxPixSearch;

        // project to arbitrary depth to get direction.
        Vector3 ptpMax = pr + hostToFrame_Kt*0.01;
        distortedProjectedPtpMax = DistortedVector2d(ptpMax.hnormalized());

        Vector2 direction = distortedProjectedPtpMax - distortedProjectedPtpMin;
        scalar_t inverseDistance = 1.0 / direction.norm();

        distortedProjectedPtpMax = DistortedVector2d(Vector2(
                distortedProjectedPtpMin.x() + pixelInterval * direction.x() * inverseDistance,
                distortedProjectedPtpMin.y() + pixelInterval * direction.y() * inverseDistance
        ));
        //undistortedProjectedPtpMax = frame->undistort(distortedProjectedPtpMax, level);

        if (!frame->isInside(distortedProjectedPtpMax, level, 5)) {
            self->lastTraceUV = Vector2(-1, -1);
            self->lastTracePixelInterval = 0;
            self->lastTraceStatus = IPS_OOB;
            assertDeterministic("OOB because max not in frame (not finite)");
            return IPS_OOB;
        }

    }

    /// set OOB if scale change too big.
    if(!(self->iDepthMin<0 || (ptpMin[2]>0.75 && ptpMin[2]<1.5))) {
        self->lastTraceUV = Vector2(-1, -1);
        self->lastTracePixelInterval = 0;
        self->lastTraceStatus = IPS_OOB;
        assertDeterministic("OOB because scale change too big");
        return IPS_OOB;
    }


    /// compute error-bounds on result in pixel. if the new interval is not at least 1/2 of the old, SKIP
    scalar_t dx = mSettingTraceSetpSize.f() * (distortedProjectedPtpMax.x() - distortedProjectedPtpMin.x());
    scalar_t dy = mSettingTraceSetpSize.f() * (distortedProjectedPtpMax.y() - distortedProjectedPtpMin.y());

    scalar_t a = Vector2(dx,dy).transpose().dot(self->gradH * Vector2(dx,dy));
    scalar_t b = Vector2(dy,-dx).transpose().dot(self->gradH * Vector2(dy,-dx));
    scalar_t errorInPixel = 0.2f + 0.2f * (a+b) / a;

    if(errorInPixel * mSettingTraceMinImprovementFactor.f() > pixelInterval && std::isfinite(self->iDepthMax))
    {
        self->lastTraceUV = (distortedProjectedPtpMax + distortedProjectedPtpMin) / 2.0;
        self->lastTracePixelInterval = pixelInterval;
        self->lastTraceStatus = IPS_BADCONDITION;
        return IPS_BADCONDITION;
    }

    if(errorInPixel > 10) errorInPixel = 10;

    /// do the discrete search
    dx /= pixelInterval;
    dy /= pixelInterval;

    if(pixelInterval > maxPixSearch)
    {
        distortedProjectedPtpMax.x() += maxPixSearch * dx;
        distortedProjectedPtpMax.y() += maxPixSearch * dy;
        pixelInterval = maxPixSearch;
    }

    int numSteps = 1.9999f + pixelInterval / mSettingTraceSetpSize.f();
    Matrix22 Rplane = hostToFrame_KRKi.topLeftCorner<2,2>();

    scalar_t randShift = distortedProjectedPtpMin.x() * 1000 - floor(distortedProjectedPtpMin.x() * 1000);
    float ptx = distortedProjectedPtpMin.x() - randShift * dx;
    float pty = distortedProjectedPtpMin.y() - randShift * dy;

    Vector2 rotatetPattern[mPattern.size()];
    for (size_t i = 0; i < mPattern.size(); i++) {
        rotatetPattern[i] = Rplane * mPattern[i];
    }

    if(!std::isfinite(dx) || !std::isfinite(dy))
    {
        self->lastTracePixelInterval = 0;
        self->lastTraceUV = Vector2(-1,-1);
        self->lastTraceStatus = IPS_OOB;
        assertDeterministic("OOB because dx or dy not finite");
        return IPS_OOB;
    }

    scalar_t errors[100];
    scalar_t bestU=0, bestV=0, bestEnergy=1e10;
    int bestIdx=-1;
    if (numSteps >= 100) numSteps = 99;

    for(int i=0;i<numSteps;i++)
    {
        scalar_t energy = 0;
        for(size_t idx = 0; idx < mPattern.size(); idx++)
        {
            DistortedVector2d position(ptx+rotatetPattern[idx][0], pty+rotatetPattern[idx][1]);

            if (!frame->isInside(position, level, 3)) {
                energy += 1e5;
                continue;
            }

            scalar_t hitColor = frame->getCaptureFrame().getGrayImage(level).interpolate(position.cast<float>());
            scalar_t referenceColor = mapPoint->getGrayPatch(mPattern[idx].x(), mPattern[idx].y(), level);

            scalar_t residual = hitColor - exposureTransition(referenceColor);
            scalar_t hw = fabs(residual) < mSettingHuberTH.f() ? 1 : mSettingHuberTH.f() / fabs(residual);
            energy += hw *residual*residual*(2-hw);
        }

        errors[i] = energy;
        if(energy < bestEnergy)
        {
            bestU = ptx;
            bestV = pty;
            bestEnergy = energy;
            bestIdx = i;
        }

        ptx += dx;
        pty += dy;
    }


    // find best score outside a +-2px radius.
    scalar_t secondBest = 1e10;
    for (int i = 0; i < numSteps; i++)
    {
        if((i < bestIdx - mMinTraceTestRadius.f() || i > bestIdx + mMinTraceTestRadius.f() ) && errors[i] < secondBest) {
            secondBest = errors[i];
        }
    }
    scalar_t newQuality = secondBest / bestEnergy;
    if(newQuality < self->quality || numSteps > 10) self->quality = newQuality;

    if(bestEnergy >= self->energyTH * mSettingsExtraSlackOnTH.f()) {
        self->lastTracePixelInterval=0;
        self->lastTraceUV = Vector2(-1,-1);
        if(self->lastTraceStatus == IPS_OUTLIER) {
            self->lastTraceStatus = IPS_OOB;
            assertDeterministic("OOB because two times outliers");
            return IPS_OOB;
        }
        else {
            self->lastTraceStatus = IPS_OUTLIER;
            return IPS_OUTLIER;
        }
    }

    // ============== set new interval ===================
    if (dx * dx > dy * dy)
    {
        self->iDepthMin = (pr[2] * (bestU-errorInPixel*dx) - pr[0]) / (hostToFrame_Kt[0] - hostToFrame_Kt[2] * (bestU-errorInPixel*dx));
        self->iDepthMax = (pr[2] * (bestU+errorInPixel*dx) - pr[0]) / (hostToFrame_Kt[0] - hostToFrame_Kt[2] * (bestU+errorInPixel*dx));
    }
    else
    {
        self->iDepthMin = (pr[2]*(bestV-errorInPixel*dy) - pr[1]) / (hostToFrame_Kt[1] - hostToFrame_Kt[2] * (bestV-errorInPixel*dy));
        self->iDepthMax = (pr[2]*(bestV+errorInPixel*dy) - pr[1]) / (hostToFrame_Kt[1] - hostToFrame_Kt[2] * (bestV+errorInPixel*dy));
    }

    if(self->iDepthMin > self->iDepthMax) std::swap(self->iDepthMin, self->iDepthMax);

    scalar_t depth = 1.0 / ((self->iDepthMin + self->iDepthMax) / 2.0);


    if (depth > 0 && std::isfinite(depth)) {
        mStatisticQuality->addValue(self->quality);
        mStatisticPixelInterval->addValue(self->lastTracePixelInterval);
        mStatisticDepthInterval->addValue((1.0 / self->iDepthMin) - (1.0 / self->iDepthMax));
    }

    self->lastTracePixelInterval=2*errorInPixel;
    self->lastTraceUV = Vector2(bestU, bestV);
    self->lastTraceStatus = IPS_GOOD;
    return IPS_GOOD;
}

CML::Ptr<CML::Optimization::DSOTracerPointPrivate, CML::NonNullable> CML::Optimization::DSOTracer::getPrivateData(PPoint point) {
    return point->getPrivate().get<DSOTracerPointPrivate>(mPrivateDataInstance);
}

void CML::Optimization::DSOTracer::freePrivateData(PPoint point, std::string reason) {
    assertThrow(!point->isGroup(IMMATUREPOINT), "The point is still immature but you tried to free it");
    point->getPrivate().free(mPrivateDataInstance, reason);
}
