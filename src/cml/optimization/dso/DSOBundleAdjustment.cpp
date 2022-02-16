#include "cml/optimization/dso/DSOBundleAdjustment.h"
#include "cml/optimization/dso/DSOTracer.h"
#include "cml/optimization/Residual.h"

CML::Optimization::DSOBundleAdjustment::DSOBundleAdjustment(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent), DSOContext(parent->getMap()) {
    mMarginalizedHessian = Matrix<Dynamic, Dynamic>(CPARS + 8, CPARS + 8);
    mMarginalizedB = Vector<Dynamic>(CPARS + 8);

    mMarginalizedHessian.setZero();
    mMarginalizedB.setZero();

    mScaledVarTH.subscribeObserver(this);
    mAbsVarTH.subscribeObserver(this);
    mMinRelBS.subscribeObserver(this);

    mTriangulator = new Hartley2003Triangulation(this);
}

void CML::Optimization::DSOBundleAdjustment::createResidual(PFrame frame, PPoint point) {

    if (frame == point->getReferenceFrame()) {
        return;
    }

    DSOResidual *r = new DSOResidual(OptimizationPair(frame, point, DIRECT));

    auto pointData = get(point);
    auto hostData = get(point->getReferenceFrame());
    auto targetData = get(frame);
    auto refcorner = point->getReferenceCorner().point0();

    Camera hostToTarget = cameraOf(hostData->get_worldToCam_evalPT()).to(cameraOf(targetData->get_worldToCam_evalPT()));
    Vector3 projectedcurp = hostToTarget.getRotationMatrix() * refcorner.homogeneous() + hostToTarget.getTranslation() * point->getReferenceInverseDepth();
    DistortedVector2d projectedcurp_Distorted = frame->distort(UndistortedVector2d(projectedcurp.hnormalized()), 0);
    scalar_t drescale = 1.0 / projectedcurp[2];

    scalar_t new_idepth = drescale * point->getReferenceInverseDepth();
    scalar_t Ku = projectedcurp_Distorted.x();
    scalar_t Kv = projectedcurp_Distorted.y();

    r->setCenterProjectedTo(Vector3(Ku, Kv, new_idepth));

    if (frame->isInside(projectedcurp_Distorted, 0, 0)) {
        r->state_NewEnergy = r->state_energy = 0;
        r->setNewState(DSORES_OUTLIER);
        r->setState(DSORES_IN);
    } else {
        r->state_NewEnergy = r->state_energy = 0;
        r->setNewState(DSORES_OUTLIER);
        r->setState(DSORES_OOB);
    }

    // point->setGroup(IMMATUREPOINT, false);
    // addPoint(point);
    addResidual(r);

    if (r->elements.frame == getFrames().back()) {
        pointData->setLastResidual(0, r, r->getState());
    } else if (getFrames().size() >= 2 && r->elements.frame == getFrames()[getFrames().size() - 2]) {
        pointData->setLastResidual(1, r, r->getState());
    }

}

void CML::Optimization::DSOBundleAdjustment::addPoints(const Set<PPoint, Hasher>& points) {

    for (auto point : points) {

        assertThrow(!have(point), "The point is already added !");

        addPoint(point);

        auto pointData = get(point);
        pointData->resetLastResidual();
        pointData->idepth_zero = point->getReferenceInverseDepth();
        pointData->status = DSOPT_ACTIVE;
        pointData->hasDepthPrior = point->getReferenceFrame()->isGroup(getMap().INITFRAME);

        for(auto frame : getFrames()) {
            createResidual(frame, point);
        }

        pointData->gradH.setZero();
        pointData->weights.resize(8);

        DistortedVector2d distortedReferenceCorner = point->getReferenceCorner().point(0);
        for (size_t i = 0; i < 8; i++) {
            const Vector2 &shift = PredefinedPattern::star8(i);
            Vector2 grad = point->getReferenceFrame()->getCaptureFrame().getDerivativeImage(0).interpolate(Vector2f(distortedReferenceCorner.x() + shift.x(), distortedReferenceCorner.y() + shift.y())).tail<2>().cast<scalar_t>();
            pointData->gradH += grad * grad.transpose();
            pointData->weights[i] = sqrt(mSettingOutlierTHSumComponent.f() / (mSettingOutlierTHSumComponent.f() + grad.squaredNorm()));
        }

    }

}

void CML::Optimization::DSOBundleAdjustment::addNewFrame(PFrame frame, int immatureGroup) {

    if (!mCalibHaveZero) {
        mCalibZero = frame->getCaptureFrame().getInternalCalibration().getParameters(0);
        mCalibHaveZero = true;
        mPinhole = frame->getCalibration().getPinhole(0);
        mWidth = frame->getWidth(0);
        mHeight = frame->getHeight(0);
    }

    // =========================== Flag Frames to be Marginalized. =========================
    flagFramesForMarginalization(frame, immatureGroup);


    // =========================== add New Frame to Hessian Struct. =========================
    addFrame(frame);
    auto frameToTrackData = get(frame);
    frameToTrackData->setEvalPT_scaled(Sophus::SE3<CML::scalar_t>(frame->getCamera().getQuaternion().toEigen(), frame->getCamera().getTranslation()), frame->getExposure(), mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());

    get(frame)->ab_exposure = frame->getCaptureFrame().getExposureTime();

    //assert(mMarginalizedHessian.cols() == 8*getFrames().size()+CPARS-8);
	mMarginalizedB.conservativeResize(8*getFrames().size()+CPARS);
    mMarginalizedHessian.conservativeResize(8*getFrames().size()+CPARS,8*getFrames().size()+CPARS);
    mMarginalizedB.tail<8>().setZero();
    mMarginalizedHessian.rightCols<8>().setZero();
    mMarginalizedHessian.bottomRows<8>().setZero();


    mAdjointsValid = false;
    mDeltaValid = false;
    mIndicesValid = false;

    computeAdjoints();
    computeDelta();
    makeIDX();


    // =========================== add new residuals for old points =========================
    for (auto point : getPoints()) {

        createResidual(frame, point);

    }

}

void CML::Optimization::DSOBundleAdjustment::marginalizeFrame(PFrame frame) {
    assert(mDeltaValid);
    assert(mAdjointsValid);
    assert(mIndicesValid);

    bool frameFound = false;
    for (auto f : getFrames()) {
        if (f == frame) {
            frameFound = true;
            break;
        }
    }

    assertThrow(frameFound, "The frame is already marginalized ?");


    //assert((int)fh->points.size()==0);
    int ndim = getFrames().size()*8+CPARS-8;// new dimension
    int odim = getFrames().size()*8+CPARS;// old dimension


    auto frameData = get(frame);


    if((int)frameData->id != (int)getFrames().size()-1)
    {
        int io = frameData->id*8+CPARS;	// index of frame to move to end
        int ntail = 8*(getFrames().size()-frameData->id-1);
        assert((io+8+ntail) == (int)getFrames().size()*8+CPARS);

        Vector8 bTmp = mMarginalizedB.segment<8>(io);
        Vector<Dynamic> tailTMP = mMarginalizedB.tail(ntail);
        mMarginalizedB.segment(io,ntail) = tailTMP;
        mMarginalizedB.tail<8>() = bTmp;

        Matrix<Dynamic, Dynamic> HtmpCol = mMarginalizedHessian.block(0,io,odim,8);
        Matrix<Dynamic, Dynamic> rightColsTmp = mMarginalizedHessian.rightCols(ntail);
        mMarginalizedHessian.block(0,io,odim,ntail) = rightColsTmp;
        mMarginalizedHessian.rightCols(8) = HtmpCol;

        Matrix<Dynamic, Dynamic> HtmpRow = mMarginalizedHessian.block(io,0,8,odim);
        Matrix<Dynamic, Dynamic> botRowsTmp = mMarginalizedHessian.bottomRows(ntail);
        mMarginalizedHessian.block(io,0,ntail,odim) = botRowsTmp;
        mMarginalizedHessian.bottomRows(8) = HtmpRow;
    }


//	// marginalize. First add prior here, instead of to active.
    mMarginalizedHessian.bottomRightCorner<8,8>().diagonal() += frameData->prior.cast<scalar_t>();
    mMarginalizedB.tail<8>() += frameData->prior.cwiseProduct(frameData->delta_prior).cast<scalar_t>();



//	std::cout << std::setprecision(16) << "HMPre:\n" << HM << "\n\n";


    Vector<Dynamic> SVec = (mMarginalizedHessian.diagonal().cwiseAbs() + Vector<Dynamic>::Constant(mMarginalizedHessian.cols(), 10)).cwiseSqrt();
    Vector<Dynamic> SVecI = SVec.cwiseInverse();


//	std::cout << std::setprecision(16) << "SVec: " << SVec.transpose() << "\n\n";
//	std::cout << std::setprecision(16) << "SVecI: " << SVecI.transpose() << "\n\n";

    // scale!
    Matrix<Dynamic, Dynamic> HMScaled = SVecI.asDiagonal() * mMarginalizedHessian * SVecI.asDiagonal();
    Vector<Dynamic> bMScaled =  SVecI.asDiagonal() * mMarginalizedB;

    // invert bottom part!
    Matrix<8, 8> hpi = HMScaled.bottomRightCorner<8,8>();
    hpi = 0.5f*(hpi+hpi);
    hpi = hpi.inverse();
    hpi = 0.5f*(hpi+hpi);

    // schur-complement!
    Matrix<Dynamic, Dynamic> bli = HMScaled.bottomLeftCorner(8,ndim).transpose() * hpi;
    HMScaled.topLeftCorner(ndim,ndim).noalias() -= bli * HMScaled.bottomLeftCorner(8,ndim);
    bMScaled.head(ndim).noalias() -= bli*bMScaled.tail<8>();

    //unscale!
    HMScaled = SVec.asDiagonal() * HMScaled * SVec.asDiagonal();
    bMScaled = SVec.asDiagonal() * bMScaled;

    // set.
    mMarginalizedHessian = 0.5*(HMScaled.topLeftCorner(ndim,ndim) + HMScaled.topLeftCorner(ndim,ndim).transpose());
    mMarginalizedB = bMScaled.head(ndim);

    // remove from vector, without changing the order!
    removeFrame(frame);


    assert((int)getFrames().size()*8+CPARS == (int)mMarginalizedHessian.rows());
    assert((int)getFrames().size()*8+CPARS == (int)mMarginalizedHessian.cols());
    assert((int)getFrames().size()*8+CPARS == (int)mMarginalizedB.size());




//	Vector<Dynamic> eigenvaluesPost = HM.eigenvalues().real();
//	std::sort(eigenvaluesPost.data(), eigenvaluesPost.data()+eigenvaluesPost.size());

//	std::cout << std::setprecision(16) << "HMPost:\n" << HM << "\n\n";

//	std::cout << "EigPre:: " << eigenvaluesPre.transpose() << "\n";
//	std::cout << "EigPost: " << eigenvaluesPost.transpose() << "\n";

    mIndicesValid = false;
    mAdjointsValid = false;
    mDeltaValid = false;

    makeIDX();

    // drop all observations of existing points in that frame.

    /* this does nothing
    for (auto r : getResiduals()) {

        if (r->elements.mapPoint->getReferenceFrame() == frame) {
            continue;
        }

        if (r->elements.frame == frame) {

            auto ph = get(r->elements.mapPoint);

            if(ph->getLastResidual(0).first == r)
                ph->getLastResidual(0).first=0;
            else if(ph->getLastResidual(1).first == r)
                ph->getLastResidual(1).first=0;

        }

    }
     */

    computeDelta();
    computeAdjoints();

}

void CML::Optimization::DSOBundleAdjustment::flagFramesForMarginalization(PFrame newFrame, int immatureGroup)
{

    int flagged = 0;
    // marginalize all frames that have not enough points.
    for(int i=0;i<(int)getFrames().size();i++)
    {

        PFrame frame = getFrames()[i];
        auto frameData = get(frame);

        PFrame frameBack = getFrames().back();
        //auto frameBackData = get(frameBack);

        scalar_t in = frameData->getResiduals().size() + frame->getReferenceGroupMapPoints(immatureGroup).size();
        scalar_t out = frameData->getNumMarginalized() + frameData->getNumResidualsOut();


        Vector2 refToFh = frameBack->getExposure().to(frame->getExposure()).getParameters();

        // TODOOOOOOOOOOOOOOOOOOOOOOO
        scalar_t setting_minPointsRemaining = 0.05;  // marg a frame if less than X% points remain.
        scalar_t setting_maxLogAffFacInWindow = 0.7; // marg a frame if factor between intensities to current frame is larger than 1/X or X.
        int   setting_minFrames = maxFrames.i() - 2; // min frames in window.

        bool notEnoughPointRemaining = in < setting_minPointsRemaining * ( in + out );
        bool tooBigFactorBetweenIntensities = fabs(log(refToFh[0])) > setting_maxLogAffFacInWindow && (int)getFrames().size() - flagged > setting_minFrames;

        if(notEnoughPointRemaining || tooBigFactorBetweenIntensities)
        {
            int num = 0;
            for (auto point : getPoints()) {
                if (point->getReferenceFrame() == frame) {
                    num++;
                }
            }
            logger.debug("Marginalization of a frame ( because of in/out ) with " + std::to_string(num) + " active points");
            frameData->flaggedForMarginalization = true;
            flagged++;
        }

    }



    // marginalize one.
    if((int)getFrames().size() - flagged >= maxFrames.i())
    {
        double smallestScore = 1;
        OptPFrame toMarginalize = nullptr;
        OptPFrame latest = getFrames().back();

        auto latestData = get(latest);

        for(PFrame reference : getFrames())
        {
            auto referenceData = get(reference);

            if(referenceData->keyid > latestData->keyid - minFrameAge.i() || referenceData->keyid == 0) {
                logger.debug("Not computing distance score because reference id (" + std::to_string(referenceData->keyid) + ") > last id (" + std::to_string(latestData->keyid) + ") - " + std::to_string(minFrameAge.i()));
                continue;
            }
            //if(fh==frameHessians.front() == 0) continue;

            double distScore = 0;
            for(PFrame target : getFrames())
            {
                if (reference == target) {
                    continue;
                }

                auto targetData = get(target);

                if (targetData->keyid > latestData->keyid - minFrameAge.i() + 1) {
                    continue;
                }
                double distanceLL = reference->getCamera().to(target->getCamera()).getTranslation().norm();
                distScore += 1.0 / (1e-5 + distanceLL);

            }
            double distanceLLBack = reference->getCamera().to(getFrames().back()->getCamera()).getTranslation().norm();
            distScore *= -sqrt(distanceLLBack);

            logger.debug("Distance Score : " + std::to_string(distScore));

            if(distScore < smallestScore)
            {
                smallestScore = distScore;
                toMarginalize = reference;
            }
        }

//		printf("MARGINALIZE frame %d, as it is the closest (score %.2f)!\n",
//				toMarginalize->frameID, smallestScore);
        int num = 0;
        for (auto point : getPoints()) {
            if (point->getReferenceFrame() == toMarginalize) {
                num++;
            }
        }
        logger.debug("Marginalization of a frame ( because of score ) " + std::to_string(num) + " active points");


        get(toMarginalize)->flaggedForMarginalization = true;
        flagged++;
    }

//	printf("FRAMES LEFT: ");
//	for(FrameHessian* fh : frameHessians)
//		printf("%d ", fh->frameID);
//	printf("\n");


}

CML::List<CML::PFrame> CML::Optimization::DSOBundleAdjustment::marginalizeFrames() {

    List<PFrame> frameToMarginalize;

    for (auto frame : getFrames()) {

        auto frameData = get(frame);
        if (frameData->flaggedForMarginalization) {

            frameToMarginalize.emplace_back(frame);


        }

    }


    for (auto frame : frameToMarginalize) {
        marginalizeFrame(frame);
    }

    return frameToMarginalize;


}

bool CML::Optimization::DSOBundleAdjustment::run(bool updatePointsOnly) {

    if (updatePointsOnly) {
        logger.important("DSO Bundle adjustment run in update points only");
    }

    Timer timer;
    timer.start();

    for (auto frame : getFrames()) {
        updateCamera(frame);
    }

    mOutliers = Set<PPoint, Hasher>();

    if (getPoints().empty()) {
        logger.error("No points...");
        return false;
    }

    mActiveResiduals.clear();

    for(auto r : getResiduals())
    {
        if(!r->isLinearized)
        {
            mActiveResiduals.push_back(r);
            r->resetOOB();
        }
    }

    computeAdjoints();
    computeDelta();


    Vector3 lastEnergy = linearizeAll(false);
    double lastEnergyL = calcLEnergy();
    double lastEnergyM = calcMEnergy();


    applyActiveRes(true);


    int numIterations = mNumIterations.i();

    scalar_t lambda = mFixedLambda.f();
    //Vector<Dynamic> previousX = Vector<Dynamic>::Constant(CPARS + 8 * getFrames().size(), NAN);

    mStatisticEnergyP->addValue(lastEnergy[0] / mActiveResiduals.size());
    mStatisticEnergyR->addValue(lastEnergy[1]);
    mStatisticEnergyL->addValue(lastEnergyL);
    mStatisticEnergyM->addValue(lastEnergyM);
    mStatisticEnergyTotal->addValue((lastEnergy[0] + lastEnergy[1] + lastEnergyL + lastEnergyM) / mActiveResiduals.size());

    timer.stopAndPrint("Preparation");

    //while (true) {
    for (int it = 0; it < numIterations; it++) {

        backupState();

        timer.start();
        // Solve the system, and compute the step
        if (!solveSystem(it, lambda)) {
            logger.error("DSO Bundle Adjustment failed to solve the system !");
            return false;
        }

        timer.stopAndPrint("Solve System");

        /*if (updatePointsOnly) {
            break;
        }*/

        bool canbreak = doStepFromBackup(updatePointsOnly);

        // eval new energy!
        timer.start();
        Vector3 newEnergy = linearizeAll(false);
        timer.stopAndPrint("Linearize all");
        double newEnergyL = calcLEnergy();
        double newEnergyM = calcMEnergy();

        double newTotalEnergy = newEnergy[0] +  newEnergy[1] +  newEnergyL + newEnergyM;
        double lastTotalEnergy = lastEnergy[0] + lastEnergy[1] + lastEnergyL + lastEnergyM;

        if (!std::isfinite(newTotalEnergy)) {
            if (mAbortBAOnFailture) {
                abort();
            }
            return false;
        }

        if(newTotalEnergy < lastTotalEnergy || mForceAccept.b())
        {
            logger.info("Applying bundle adjustment step. New energy = " + std::to_string(newTotalEnergy) + ". Last Total Energy = " + std::to_string(lastTotalEnergy));

            mStatisticEnergyP->addValue(newEnergy[0]);
            mStatisticEnergyR->addValue(newEnergy[1]);
            mStatisticEnergyL->addValue(newEnergyL);
            mStatisticEnergyM->addValue(newEnergyM);
            mStatisticEnergyTotal->addValue(newTotalEnergy / mActiveResiduals.size());

            applyActiveRes(true);

            lastEnergy = newEnergy;
            lastEnergyL = newEnergyL;
            lastEnergyM = newEnergyM;

            lambda *= 0.25;

        }
        else
        {
            if (!std::isfinite(newTotalEnergy)) {
                logger.error("Skipping bundle adjustment step. New energy = " + std::to_string(newTotalEnergy) +
                            ". Last Total Energy = " + std::to_string(lastTotalEnergy));
            } else {
                logger.info("Skipping bundle adjustment step. New energy = " + std::to_string(newTotalEnergy) +
                            ". Last Total Energy = " + std::to_string(lastTotalEnergy));
            }
            loadSateBackup();
            lastEnergy = linearizeAll(false);
            lastEnergyL = calcLEnergy();
            lastEnergyM = calcMEnergy();
            lambda *= 1e2;
        }


         if(canbreak && it >= 1) break;

    }

    timer.start();

    auto frameBackData = get(getFrames().back());

    Vector<10> newStateZero = Vector<10>::Zero();
    newStateZero.segment<2>(6) = frameBackData->get_state().segment<2>(6);
    frameBackData->setEvalPT(frameBackData->PRE_worldToCam, newStateZero, mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());

    mDeltaValid = false;
    mAdjointsValid = false;
    computeAdjoints();
    computeDelta();

    lastEnergy = linearizeAll(true);

    if(!std::isfinite((double)lastEnergy[0]) || !std::isfinite((double)lastEnergy[1]) || !std::isfinite((double)lastEnergy[2]))
    {
        if (mAbortBAOnFailture) {
            abort();
        }
        logger.error("Not finite energy");
        return false;
    }

    timer.stopAndPrint("Application");

    return true;
}

void CML::Optimization::DSOBundleAdjustment::backupState() {

    for (auto frame : getFrames()) {
        auto self = get(frame);
        self->backupState();
    }

    for (auto point : getPoints()) {
        auto self = get(point);
        self->idepth_backup = point->getReferenceInverseDepth();
    }

    mCalibBackup = getFrames()[0]->getCaptureFrame().getInternalCalibration().getParameters(0);

}

void CML::Optimization::DSOBundleAdjustment::loadSateBackup() {
    //getFrames()[0]->getCaptureFrame().getInternalCalibration().setParameters(mCalibBackup);

    for (auto frame : getFrames()) {
        auto self = get(frame);
        self->loadSateBackup(mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());
        frame->setCamera(cameraOf(self->PRE_worldToCam));
        frame->setExposureParameters(self->aff_g2l());
    }

    for (auto point : getPoints()) {
        auto self = get(point);
        point->setReferenceInverseDepth(self->idepth_backup);
        self->idepth_zero = self->idepth_backup;
    }

    mDeltaValid = false;
    computeDelta();
}

bool CML::Optimization::DSOBundleAdjustment::doStepFromBackup(bool fixCamera) {

    float sumA=0, sumB=0, sumT=0, sumR=0, sumID=0, numID=0;

    float sumNID=0;


    //getFrames()[0]->getCaptureFrame().getInternalCalibration().setParameters(mCalibBackup + mCalibStep);

    for(auto f : getFrames())
    {
        auto self = get(f);
        if (fixCamera) {
            auto step = self->getStep();
            step.head<6>().setZero();
            self->setStep(step);
        }
        self->doStepFromBackup(mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());
        f->setCamera(cameraOf(self->PRE_worldToCam));
        f->setExposureParameters(self->aff_g2l());

        sumA += self->getStep()[6]*self->getStep()[6];
        sumB += self->getStep()[7]*self->getStep()[7];
        sumT += self->getStep().segment<3>(0).squaredNorm();
        sumR += self->getStep().segment<3>(3).squaredNorm();

    }

    for (auto p : getPoints()) {

        auto self = get(p);

        scalar_t newidepth = self->idepth_backup + self->step;
        if (std::isfinite(newidepth) && newidepth > 0) {
            p->setReferenceInverseDepth(newidepth);
        } else {
            // mOutliers.insert(p);
            continue;
        }

        sumID += self->step * self->step;
        sumNID += fabs(self->idepth_backup);
        numID++;

        self->idepth_zero = p->getReferenceInverseDepth();

    }

    for (auto p : mIndirectPointToOptimizeSet) {
        List<PFrame> frames;
        for (auto frame : getFrames()) {
            if (frame->getFeaturePoint(p).has_value()) {
                ReprojectionError re(frame, p);
                scalar_t res;
                re.error(res);
                if (res > 0.01) {
                    frame->removeMapPoint(p);
                } else {
                    frames.emplace_back(frame);
                }
            }
        }
        mTriangulator->triangulateNCam(p, p->getIndirectApparitions());
    }

    sumA /= getFrames().size();
    sumB /= getFrames().size();
    sumR /= getFrames().size();
    sumT /= getFrames().size();
    sumID /= numID;
    sumNID /= numID;

    mDeltaValid = false;
    computeDelta();

    return sqrtf(sumA) < 0.0005 * mThOptIterations.f() &&
           sqrtf(sumB) < 0.00005 * mThOptIterations.f() &&
           sqrtf(sumR) < 0.00005 * mThOptIterations.f() &&
           sqrtf(sumT)*sumNID < 0.00005 * mThOptIterations.f();

}

void CML::Optimization::DSOBundleAdjustment::computeAdjoints() {

    mAdHost.resize(getFrames().size() * getFrames().size());
    mAdTarget.resize(getFrames().size() * getFrames().size());

    mAccumulatorActive.resize(getFrames().size() * getFrames().size());
    mAccumulatorLinearized.resize(getFrames().size() * getFrames().size());

    mAccE.resize(getFrames().size() * getFrames().size());
    mAccEB.resize(getFrames().size() * getFrames().size());
    mAccD.resize(getFrames().size() * getFrames().size() * getFrames().size());



    for (size_t i = 0; i < getFrames().size() * getFrames().size(); i++) {
        mAccumulatorActive[i].initialize();
        mAccumulatorLinearized[i].initialize();
    }

    mAccbc.initialize();
    mAccHcc.initialize();

    for(size_t i = 0; i < getFrames().size() * getFrames().size(); i++)
    {
        mAccE[i].initialize();
        mAccEB[i].initialize();

        for(size_t j = 0; j < getFrames().size(); j++) {
            mAccD[i * getFrames().size() + j].initialize();
        }
    }

    for (size_t h = 0; h < getFrames().size(); h++) {
        for (size_t t = 0; t < getFrames().size(); t++) {

            PFrame host = getFrames()[h];
            PFrame target = getFrames()[t];

            auto hostData = get(host);
            auto targetData = get(target);

            SE3 hostToTarget = targetData->get_worldToCam_evalPT() * hostData->get_worldToCam_evalPT().inverse();
            Vector2 lightHostToTarget = hostData->aff_g2l_0(mScaleLightA.f(), mScaleLightB.f()).to(targetData->aff_g2l_0(mScaleLightA.f(), mScaleLightB.f())).getParameters();

            Matrix<8, 8> AH = Matrix<8, 8>::Identity();
            Matrix<8, 8> AT = Matrix<8, 8>::Identity();

            AH.topLeftCorner<6,6>() = -hostToTarget.Adj().transpose();
            AT.topLeftCorner<6,6>() = Matrix<6, 6>::Identity();

            AT(6,6) = -lightHostToTarget[0];
            AH(6,6) = lightHostToTarget[0];
            AT(7,7) = -1;
            AH(7,7) = lightHostToTarget[0];

            AH.block<3,8>(0,0) *= mScaleTranslation.f();
            AH.block<3,8>(3,0) *= mScaleRotation.f();
            AH.block<1,8>(6,0) *= mScaleLightA.f();
            AH.block<1,8>(7,0) *= mScaleLightB.f();
            AT.block<3,8>(0,0) *= mScaleTranslation.f();
            AT.block<3,8>(3,0) *= mScaleRotation.f();
            AT.block<1,8>(6,0) *= mScaleLightA.f();
            AT.block<1,8>(7,0) *= mScaleLightB.f();

            mAdHost[h+t*getFrames().size()] = AH;
            mAdTarget[h+t*getFrames().size()] = AT;
        }
    }

    mAdjointsValid = true;

}

void CML::Optimization::DSOBundleAdjustment::computeDelta() {

    mAdHTdeltaF.resize(getFrames().size() * getFrames().size());
    for(size_t h = 0; h < getFrames().size(); h++) {
        auto selfHost = get(getFrames()[h]);
        for (size_t t = 0; t < getFrames().size(); t++) {
            auto selfTarget = get(getFrames()[t]);

            int idx = h + t * getFrames().size();

            mAdHTdeltaF[idx] = selfHost->get_state_minus_stateZero().head<8>().transpose() * mAdHost[idx]
            + selfTarget->get_state_minus_stateZero().head<8>().transpose() * mAdTarget[idx];
        }
    }

    mCDeltaF = (getFrames()[0]->getCaptureFrame().getInternalCalibration().getParameters(0) - mCalibZero);

    for(auto frame : getFrames())
    {
        auto selfHost = get(frame);

        {
            // TODO OOOOOOOOOOOOOOOOOOOOOOOO

            float fac = 1.0;

            float setting_initialRotPrior = 1e11*fac;
            float setting_initialTransPrior = 1e10*fac;
            float setting_initialAffBPrior = 1e14*fac;
            float setting_initialAffAPrior = 1e14*fac;

            float setting_affineOptModeA = 1e12*fac; //-1: fix. >=0: optimize (with prior, if > 0).
            float setting_affineOptModeB = 1e8*fac; //-1: fix. >=0: optimize (with prior, if > 0).

            if (!mOptimizeA.b()) {
                setting_affineOptModeA = -1;
            }
            if (!mOptimizeB.b()) {
                setting_affineOptModeB = -1;
            }

            // TODO OOOOOOOOOOOOOOOOOOOOOOOO

            Vector<8> p = Vector<8>::Zero();
            if(selfHost->keyid == 0) // If this is the first frame in the map
            {
                p.head<3>() = Vector3::Constant(setting_initialTransPrior);
                p.segment<3>(3) = Vector3::Constant(setting_initialRotPrior);

                p[6] = setting_initialAffAPrior;
                p[7] = setting_initialAffBPrior;
            }
            else
            {
                if(setting_affineOptModeA < 0)
                    p[6] = setting_initialAffAPrior;
                else
                    p[6] = setting_affineOptModeA;

                if(setting_affineOptModeB < 0)
                    p[7] = setting_initialAffBPrior;
                else
                    p[7] = setting_affineOptModeB;
            }
            // p[8] = setting_initialAffAPrior;
            // p[9] = setting_initialAffBPrior;
            selfHost->prior = p;
        }

        selfHost->delta = selfHost->get_state_minus_stateZero().head<8>();
     //   assertThrow(selfHost->get_state()(0) < 0.01, "WTF");
     //   assertThrow(selfHost->delta.norm() < 0.03, "Delta too large");
        selfHost->delta_prior = (selfHost->get_state() - selfHost->prior_zero).head<8>();

    }

    for (auto point : getPoints()) {
        // for the reference frame
        auto self = get(point);
        self->priorF = self->hasDepthPrior ? mIdepthFixPrior.i() : 0;
        self->deltaF  = point->getReferenceInverseDepth() - self->idepth_zero;
    }


    Vector<Dynamic> delta = Vector<Dynamic>(4 + getFrames().size() * 8);
    delta.head<4>() = mCDeltaF.cast<scalar_t>();
    for(size_t h = 0; h < getFrames().size(); h++) delta.segment<8>(4 + 8 * h) = get(getFrames()[h])->delta;
   // assertThrow(delta.norm() < 0.03, "Delta too big inside computeDelta" + std::to_string(delta.norm()));


    mDeltaValid = true;
}

void CML::Optimization::DSOBundleAdjustment::orthogonalize(Vector<Dynamic>& b)
{
//	Vector<Dynamic> eigenvaluesPre = H.eigenvalues().real();
//	std::sort(eigenvaluesPre.data(), eigenvaluesPre.data()+eigenvaluesPre.size());
//	std::cout << "EigPre:: " << eigenvaluesPre.transpose() << "\n";


    // decide to which nullspaces to orthogonalize.
    std::vector<Vector<Dynamic>> ns;
    ns.insert(ns.end(), mLastNullspaces_pose.begin(), mLastNullspaces_pose.end());
    ns.insert(ns.end(), mLastNullspaces_scale.begin(), mLastNullspaces_scale.end());
//	if(setting_affineOptModeA <= 0)
//		ns.insert(ns.end(), lastmLastNullspaces_affA.begin(), lastmLastNullspaces_affA.end());
//	if(setting_affineOptModeB <= 0)
//		ns.insert(ns.end(), lastmLastNullspaces_affB.begin(), lastmLastNullspaces_affB.end());





    // make Nullspaces matrix
    Matrix<Dynamic, Dynamic> N(ns[0].rows(), ns.size());
    for(unsigned int i=0;i<ns.size();i++) {
        N.col(i) = ns[i].normalized().cast<scalar_t>();
    }



    // compute Npi := N * (N' * N)^-1 = pseudo inverse of N.
    Eigen::JacobiSVD<Matrix<Dynamic, Dynamic>> svdNN(N, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Vector<Dynamic> SNN = svdNN.singularValues();
    double minSv = 1e10, maxSv = 0;
    for(int i=0;i<SNN.size();i++)
    {
        if(SNN[i] < minSv) {
            minSv = SNN[i];
        }
        if(SNN[i] > maxSv) {
            maxSv = SNN[i];
        }
    }
    for(int i=0;i<SNN.size();i++)
    {
        if(SNN[i] > mSolverModeDelta.f() * maxSv) {
            SNN[i] = 1.0 / SNN[i];
        } else {
            SNN[i] = 0;
        }
    }

    Matrix<Dynamic, Dynamic> Npi = svdNN.matrixU() * SNN.asDiagonal() * svdNN.matrixV().transpose(); 	// [dim] x 9.
    Matrix<Dynamic, Dynamic> NNpiT = N*Npi.transpose(); 	// [dim] x [dim].
    Matrix<Dynamic, Dynamic> NNpiTS = 0.5*(NNpiT + NNpiT.transpose());	// = N * (N' * N)^-1 * N'.

    b -= NNpiTS * b;
    // H -= NNpiTS * H * NNpiTS;


//	std::cout << std::setprecision(16) << "Orth SV: " << SNN.reverse().transpose() << "\n";

//	Vector<Dynamic> eigenvaluesPost = H.eigenvalues().real();
//	std::sort(eigenvaluesPost.data(), eigenvaluesPost.data()+eigenvaluesPost.size());
//	std::cout << "EigPost:: " << eigenvaluesPost.transpose() << "\n";

}

void CML::Optimization::DSOBundleAdjustment::setZero() {
    mAccHcc.initialize();
    mAccbc.initialize();

    for (size_t i = 0; i < getFrames().size() * getFrames().size(); i++) {

        mAccumulatorActive[i].initialize();
        mAccumulatorLinearized[i].initialize();

        mAccE[i].initialize();
        mAccEB[i].initialize();

    }

    for (size_t i = 0; i < getFrames().size() * getFrames().size() * getFrames().size(); i++) {

        mAccD[i].initialize();

    }
}

CML::Vector<CML::Dynamic> CML::Optimization::DSOBundleAdjustment::solveLevenbergMarquardt(const Matrix<Dynamic, Dynamic> &HL_top,
                                                                                                         const Matrix<Dynamic, Dynamic> &HA_top,
                                                                                                         const Matrix<Dynamic, Dynamic> &HM_top,
                                                                                                         const Matrix<Dynamic, Dynamic> &H_sc,
                                                                                                         const Vector<Dynamic> &bL_top,
                                                                                                         const Vector<Dynamic> &bA_top,
                                                                                                         const Vector<Dynamic> &bM_top,
                                                                                                         const Vector<Dynamic> &b_sc,
                                                                                                         double lambda,
                                                                                                         bool mustOrthogonalize) {

    // Compute final hessian
    Matrix<Dynamic, Dynamic> HFinal_top;
    Vector<Dynamic> bFinal_top;

    HFinal_top = HL_top + HM_top + HA_top;
    bFinal_top = bL_top + bM_top + bA_top - b_sc;


    //lastHS = HFinal_top - H_sc;
    //lastbS = bFinal_top;

    for(size_t i = 0; i < 8 * getFrames().size() + CPARS; i++) {
        HFinal_top(i,i) *= (1+lambda);
    }
    HFinal_top -= H_sc * (1.0f/(1+lambda));

    // Solve the system
    Vector<Dynamic> SVecI = (HFinal_top.diagonal() + Vector<Dynamic>::Constant(HFinal_top.cols(), 10)).cwiseSqrt().cwiseInverse();
    Matrix<Dynamic, Dynamic> HFinalScaled = SVecI.asDiagonal() * HFinal_top * SVecI.asDiagonal();
    Vector<Dynamic> x = Vector<Dynamic>::Zero(HFinalScaled.rows());

    if (mOptimizeCalibration.b()) {
        x = SVecI.asDiagonal() * HFinalScaled.ldlt().solve(SVecI.asDiagonal() * bFinal_top);//  SVec.asDiagonal() * svd.matrixV() * Ub;
    } else {
        x.tail(HFinalScaled.rows() - 4) = SVecI.tail(SVecI.rows()-4).asDiagonal() * HFinalScaled.block(4,4,HFinalScaled.rows()-4,HFinalScaled.cols()-4).ldlt().solve(SVecI.tail(SVecI.rows()-4).asDiagonal() * bFinal_top.tail(bFinal_top.rows() - 4));
    }


        if (getFrames().size() > 4) {
            addIndirectToProblem(x);
        }

    // Orthogonalize x later
    if (mustOrthogonalize) {
        orthogonalize(x);
    }

    return x;
}

bool CML::Optimization::DSOBundleAdjustment::solveSystem(int iteration, double lambda) {

    //tex:
    // Solve the gauss-newton system as $\textbf{H} = \textbf{J}^{\top}\textbf{W}\textbf{J}$ and $b=-\textbf{J}^{\top}\textbf{W}\textbf{r}$
    // Where $\textbf{J}$ is the jacobian, $\textbf{r}$ the residual vector, and $\textbf{W}$ is the diagonal matrix containing the wieghts.

    //tex:
    // We solve one system for all the camera parameters, and one problem per points.
    // To solve $\textbf{H} \textbf{x} = \textbf{b}$ for the camera parameters,
    // we decompose $\textbf{H}$ and $\textbf{b}$ for the active points, the linearized points, the marginalized points, and the schur complement.

    assertThrow(mDeltaValid, "Delta not computed");
    assertThrow(mAdjointsValid, "Adjoints not computed");
    assertThrow(mIndicesValid, "Indices not computed");

    setZero();
    computeNullspaces();

    if (mFixLambda.b()) {
        lambda = mFixedLambda.f();
    }

    List<Pair<PPoint, Ptr<DSOPoint, NonNullable>>> points = getPointsAsList();

    // accumulateAF_MT
    //#pragma omp parallel
    {

        #pragma omp for
        for (int i = 0; i < points.size(); i++) {
            addToHessianTop(points[i].first, points[i].second, DSORES_ACTIVE);
        }
        stitchDoubleTop(mAccumulatorActive, HA_top, bA_top, false);

        // accumulateLF_MT
        #pragma omp for
        for (int i = 0; i < points.size(); i++) {
            addToHessianTop(points[i].first, points[i].second, DSORES_LINEARIZED);
        }
        stitchDoubleTop(mAccumulatorLinearized, HL_top, bL_top, true);

        // accumulateSCF_MT
        #pragma omp for
        for (int i = 0; i < points.size(); i++) {
            addToHessianSC(points[i].first, points[i].second, true);
        }
        stitchDoubleSC(H_sc, b_sc);
    }

    // stitched Delta
    Vector<Dynamic> d = Vector<Dynamic>(CPARS+getFrames().size()*8);
    d.head<CPARS>() = mCDeltaF.cast<scalar_t>();
    for(size_t h = 0; h < getFrames().size(); h++) {
        d.segment<8>(CPARS+8*h) = get(getFrames()[h])->delta.cast<scalar_t>();
    }

    // Compute bM_top
    bM_top = (mMarginalizedB + mMarginalizedHessian * d);

    // Solving
    Vector<Dynamic> x = solveLevenbergMarquardt(HL_top, HA_top, mMarginalizedHessian, H_sc, bL_top, bA_top, bM_top, b_sc, lambda, iteration >= 2).cast<scalar_t>();

    // Vector<Dynamic> xWithoutA = solveLevenbergMarquardt(HL_top, H_zero, mMarginalizedHessian, H_sc, bL_top, b_zero, bM_top, b_sc, lambda, iteration >= 2);
   // Vector<Dynamic> xWithoutL = solveLevenbergMarquardt(H_zero, HA_top, mMarginalizedHessian, H_sc, b_zero, bA_top, bM_top, b_sc, lambda, iteration >= 2);
   // Vector<Dynamic> xWithoutM = solveLevenbergMarquardt(HL_top, HA_top, H_zero, H_sc, bL_top, bA_top, b_zero, b_sc, lambda, iteration >= 2);


   // std::cout << "NORM : x / A / L / M : " << x.norm() << " " << xWithoutA.norm() << " " << xWithoutL.norm() << " " << xWithoutM.norm() << std::endl;


    // Make statistics
    mStatisticHessianP->addValue(HA_top.norm());
    mStatisticHessianL->addValue(HL_top.norm());
    mStatisticHessianM->addValue(mMarginalizedHessian.norm());
    mStatisticHessianSC->addValue(H_sc.norm());

    mStatisitcBP->addValue(bA_top.norm());
    mStatisitcBL->addValue(bL_top.norm());
    mStatisitcBM->addValue(bM_top.norm());
    mStatisitcBSC->addValue(b_sc.norm());

    mStatisticXNorm->addValue(x.norm());

    // resubstituteF_MT
    List<Matrix<1, 8>> xAd;
    xAd.resize(getFrames().size() * getFrames().size());

    mCalibStep = - x.head<4>();

    for (size_t h = 0; h < getFrames().size(); h++) {

        PFrame host = getFrames()[h];
        auto hostSelf = get(host);

        Vector<10> newStep;
        newStep.head<8>() = -x.segment<8>(4 + 8 * hostSelf->id);
        newStep.tail<2>().setZero();
        hostSelf->setStep(newStep);

        for (size_t t = 0; t < getFrames().size(); t++) {
            PFrame target = getFrames()[t];
            auto targetSelf = get(target);

            xAd[getFrames().size() * hostSelf->id + targetSelf->id] = x.segment<8>(4+8*h).transpose() * mAdHost[h+getFrames().size()*t] + x.segment<8>(4+8*t).transpose() * mAdTarget[h+getFrames().size()*t];

        }

    }

    int numPointStepNotFinite = 0;

    for (auto point : getPoints()) {
        auto self = get(point);
        auto selfHost = get(point->getReferenceFrame());

        int ngoodres = 0;
        for(auto r : self->getResiduals()) {
            if(r->isActiveAndIsGoodNEW) ngoodres++;
        }
        if(ngoodres==0)
        {
            self->step = 0;
            continue;
        }

        scalar_t b = self->bdSumF;
        b -= mCalibStep.dot(self->Hcd_accAF.cast<scalar_t>() + self->Hcd_accLF.cast<scalar_t>());

        for(auto r : self->getResiduals())
        {
            if(!r->isActiveAndIsGoodNEW) continue;

            auto selfTarget = get(r->elements.frame);

            b -= xAd[selfHost->id * getFrames().size() + selfTarget->id] * r->JpJdF.cast<scalar_t>();
        }

        self->step = -b* self->HdiF;
        // step =  cStep - bdSumF / H

        if (!std::isfinite(self->step)) {
            numPointStepNotFinite = numPointStepNotFinite + 1;
        }
    }

    if (numPointStepNotFinite > 0) {
        logger.error("DSO Bundle adjustment have " + std::to_string(numPointStepNotFinite) + " which don't have a finite step");
        return false;
    }

    return true;
}

CML::Vector3 CML::Optimization::DSOBundleAdjustment::linearizeAll(bool fixLinearization) {

    // Precomputation

    HashMap<PFrame, int, Hasher> frameToId;
    DSOFramePrecomputed precomputedArray[getFrames().size()][getFrames().size()];

    #if CML_USE_OPENMP
    #pragma omp for schedule(static) ordered
    #endif
    for (size_t i = 0; i < getFrames().size(); i++) {

        frameToId[getFrames()[i]] = i;

        for (size_t j = 0; j < getFrames().size(); j++) {

            auto hostData = get(getFrames()[i]);
            auto targetData =  get(getFrames()[j]);
            precomputedArray[i][j] = DSOFramePrecomputed(hostData.p(), targetData.p());

        }

    }


    double lastEnergyP = 0;
    double lastEnergyR = 0;
    double num = 0;


    Set<DSOResidual*> toRemove;
    Mutex toRemoveMutex;

    scalar_t stats = 0;

    logger.info("Linearizing " + std::to_string(mActiveResiduals.size()) + " residuals");

    Timer timer;
    timer.start();

    for (size_t i = 0; i < mActiveResiduals.size(); i++)
    {
        auto &r = mActiveResiduals[i];

        auto &precomputed = precomputedArray[frameToId[r->elements.mapPoint->getReferenceFrame()]][frameToId[r->elements.frame]];

        stats += linearize(r, precomputed);

        if(fixLinearization)
        {
            applyRes(r, true);

            if(r->isActiveAndIsGoodNEW)
            {
                auto p = get(r->elements.mapPoint);

                Matrix33 K = r->elements.frame->getK(0);

                Matrix33 PRE_KRKiTll = K * precomputed.PRE_RTll * K.inverse();
                Vector3 PRE_KtTll = K * precomputed.PRE_tTll;

                Vector3 ptp_inf = PRE_KRKiTll * r->elements.mapPoint->getReferenceCorner().point0().homogeneous();	// projected point assuming infinite depth.
                Vector3 ptp = ptp_inf + PRE_KtTll * r->elements.mapPoint->getReferenceInverseDepth();	// projected point with real depth.
                float relBS = 0.01*((ptp_inf.head<2>() / ptp_inf[2])-(ptp.head<2>() / ptp[2])).norm();	// 0.01 = one pixel.

                if(relBS > p->getMaxRelBaseline()) {
                    p->setMaxRelBaseline(relBS);
                    p->updatePointUncertainty(r->elements.mapPoint, mScaledVarTH.f(), mAbsVarTH.f(),
                                              mMinRelBS.f());
                }

                p->numGoodResiduals++;


            }
            else
            {
                LockGuard lg(toRemoveMutex);
                toRemove.insert(r);
            }
        }
    }

    timer.stopAndPrint("Linearization time");

    lastEnergyP = stats;

    setNewFrameEnergyTH();


    if(fixLinearization)
    {

        for(auto r : mActiveResiduals)
        {
            auto ph = get(r->elements.mapPoint);
            ph->setResidualState(r, r->getState());
        }

        int numResidualRemoveOnLastFrame = 0;
        for(auto r : toRemove)
        {
            if (r->elements.frame == getFrames()[getFrames().size() - 1] && r->getState() == DSORES_OUTLIER) {
                numResidualRemoveOnLastFrame++;
            }
            auto ph = get(r->elements.mapPoint);

            if(ph->getLastResidual(0).first == r)
                ph->getLastResidual(0).first=nullptr;
            else if(ph->getLastResidual(1).first == r)
                ph->getLastResidual(1).first=nullptr;

        }

        logger.warn("Dropping " + std::to_string(toRemove.size()) + " residuals ( " + std::to_string(numResidualRemoveOnLastFrame) + " outlier on last frame )");
        Set<PPoint, Hasher> outliers = removeResiduals(toRemove);
        logger.warn(std::to_string(outliers.size()) + " points have no residual.");
        mOutliers.insert(outliers.begin(), outliers.end());

    }

    return Vector3(lastEnergyP, lastEnergyR, num);

}

inline CML::scalar_t CML::Optimization::DSOBundleAdjustment::linearize(DSOResidual* pair, const DSOFramePrecomputed &precomputed, int level) {

    assertThrow(level == 0, "Incompatible level");

    pair->state_NewEnergyWithOutlier = -1;

    if(pair->getState() == DSORES_OOB)
    {
        pair->setState(DSORES_OOB);
        return pair->state_energy;
    }

    auto point = pair->elements.mapPoint;
    auto frameToTrack = pair->elements.frame;

    // auto hostData = get(pair->elements.mapPoint->getReferenceFrame());
    // auto targetData = get(pair->elements.frame);
    auto hostData = precomputed.mHostData;
    auto targetData = precomputed.mTargetData;
    auto pointData = unsafe_get(pair->elements.mapPoint);
    scalar_t pointIdepth = point->getReferenceInverseDepth();

    const GradientImage &image = frameToTrack->getCaptureFrame().getDerivativeImage(level);

    assertThrow(hostData->id >= 0, "Host frame invalid");
    assertThrow(targetData->id >= 0, "Target frame invalid");

    float JIdxJIdx_00 = 0, JIdxJIdx_11 = 0, JIdxJIdx_10 = 0;
    float JabJIdx_00 = 0, JabJIdx_01 = 0, JabJIdx_10 = 0, JabJIdx_11 = 0;
    float JabJab_00 = 0, JabJab_01 = 0, JabJab_11 = 0;

    float wJI2_sum = 0;

    float energyLeft = 0;

    Vector2 refcorner_Distorted_center = point->getReferenceCorner().point0();

    Matrix33 R = precomputed.trialRefToTarget.getRotationMatrix();
    Vector3 t = precomputed.trialRefToTarget.getTranslation();


    {

        Vector2 refcorner = mPinhole.undistort(refcorner_Distorted_center);

        Vector3 projectedcurp = R * refcorner.homogeneous() + t * pointIdepth;
        Vector2 projectedcurp_Distorted = mPinhole.distort((Vector2)projectedcurp.hnormalized());
        float drescale = 1.0 / projectedcurp[2];

        if (!(projectedcurp_Distorted.x() >= 2 && projectedcurp_Distorted.y() >= 2 && projectedcurp_Distorted.x() < mWidth - 2 && projectedcurp_Distorted.y() < mHeight - 2)) {
            pair->setNewState(DSORES_OOB);
            return pair->state_energy;
        }

        Vector<6> d_xi_x, d_xi_y;
        Vector<4> d_C_x, d_C_y;
        float d_d_x, d_d_y;

        float new_idepth = drescale * pointIdepth;
        float u = projectedcurp.x();
        float v = projectedcurp.y();
        float Ku = projectedcurp_Distorted.x();
        float Kv = projectedcurp_Distorted.y();
        Vector3 KliP = refcorner.homogeneous(); // todo : check this

        Matrix33 K = frameToTrack->getK(0);
        float fx = K(0,0);
        float fy = K(1,1);

        pair->setCenterProjectedTo(Vector3(Ku, Kv, new_idepth));

        // diff d_idepth
        d_d_x = drescale * (precomputed.PRE_tTll_0[0]-precomputed.PRE_tTll_0[2]*u)*fx;
        d_d_y = drescale * (precomputed.PRE_tTll_0[1]-precomputed.PRE_tTll_0[2]*v)*fy;

        // diff calib
        d_C_x[2] = drescale*(precomputed.PRE_RTll_0(2,0)*u-precomputed.PRE_RTll_0(0,0));
        d_C_x[3] = fx * drescale*(precomputed.PRE_RTll_0(2,1)*u-precomputed.PRE_RTll_0(0,1)) / fy;
        d_C_x[0] = KliP[0]*d_C_x[2];
        d_C_x[1] = KliP[1]*d_C_x[3];

        d_C_y[2] = fy * drescale*(precomputed.PRE_RTll_0(2,0)*v-precomputed.PRE_RTll_0(1,0)) / fx;
        d_C_y[3] = drescale*(precomputed.PRE_RTll_0(2,1)*v-precomputed.PRE_RTll_0(1,1));
        d_C_y[0] = KliP[0]*d_C_y[2];
        d_C_y[1] = KliP[1]*d_C_y[3];

        d_C_x[0] = (d_C_x[0]+u)* mScaleF.f();
        d_C_x[1] *=  mScaleF.f();
        d_C_x[2] = (d_C_x[2]+1)* mScaleC.f();
        d_C_x[3] *=  mScaleC.f();

        d_C_y[0] *=  mScaleF.f();
        d_C_y[1] = (d_C_y[1]+v)* mScaleF.f();
        d_C_y[2] *=  mScaleC.f();
        d_C_y[3] = (d_C_y[3]+1)* mScaleC.f();


        d_xi_x[0] = new_idepth*fx;
        d_xi_x[1] = 0;
        d_xi_x[2] = -new_idepth*u*fx;
        d_xi_x[3] = -u*v*fx;
        d_xi_x[4] = (1+u*u)*fx;
        d_xi_x[5] = -v*fx;

        d_xi_y[0] = 0;
        d_xi_y[1] = new_idepth*fy;
        d_xi_y[2] = -new_idepth*v*fy;
        d_xi_y[3] = -(1+v*v)*fy;
        d_xi_y[4] = u*v*fy;
        d_xi_y[5] = u*fy;

        pair->rJ.Jpdxi[0] = d_xi_x.cast<float>();
        pair->rJ.Jpdxi[1] = d_xi_y.cast<float>();

        pair->rJ.Jpdc[0] = d_C_x.cast<float>();
        pair->rJ.Jpdc[1] = d_C_y.cast<float>();
        pair->rJ.Jpdd[0] = d_d_x;
        pair->rJ.Jpdd[1] = d_d_y;
    }

    assertThrow(pointData->weights.size() > 0, "The point weights must be initialized");

    for (int idx = 0; idx < 8; idx++) {

        Vector2 shift = PredefinedPattern::star8(idx);

        Vector2 refcorner_Distorted = refcorner_Distorted_center + shift;
        Vector2 refcorner = mPinhole.undistort(refcorner_Distorted);
        Vector3 projectedcurp = R * refcorner.homogeneous() + t * pointIdepth;
        Vector2 projectedcurp_Distorted = mPinhole.distort((Vector2)projectedcurp.hnormalized());

        // Check that the projection is finite
        //if (!std::isfinite(projectedcurp_Distorted.x()) || !std::isfinite(projectedcurp_Distorted.y())) {
        //    pair->setNewState(DSORES_OOB);
        //    return pair->state_energy;
        //}

        // Check that the projection is inside the current frame, with a padding of 3, to allow the computation of the derivative on the image
        if (!(projectedcurp_Distorted.x() >= 2 && projectedcurp_Distorted.y() >= 2 && projectedcurp_Distorted.x() < mWidth - 2 && projectedcurp_Distorted.y() < mHeight - 2)) {
            pair->setNewState(DSORES_OOB);
            return pair->state_energy;
        }

        // scalar_t refColor = point->getGrayPatch(shift.x(), shift.y(), level);
        float refColor = pointData->colors[idx];


        auto curColorWithGradient = image.interpolate(projectedcurp_Distorted.cast<float>());

        if (!curColorWithGradient.allFinite()) {
            pair->setState(DSORES_OOB);
            return pair->state_energy;
        }

        float curColor = curColorWithGradient(0);
        Vector2f curColorGradient = curColorWithGradient.tail<2>();

        float curRealColor = curColor;
        float refRealColor = precomputed.exposureTransition(refColor);

        float residual = curRealColor - refRealColor;

        float hw = fabs(residual) < mHuberThreshold.f() ? 1 : mHuberThreshold.f() / fabsf(residual);
        float w = sqrtf(mSettingOutlierTHSumComponent.f() / (mSettingOutlierTHSumComponent.f() + curColorGradient.squaredNorm()));
        w = 0.5f*(w + pointData->weights[idx]);

        energyLeft += w * w * hw * residual * residual * ( 2.0 - hw);

        {
            if(hw < 1) hw = sqrtf(hw);
            hw = hw * w;

            float b0 = hostData->getB0(mScaleLightB.f());

            Vector3 hitColor = Vector3(curColor, curColorGradient(0) * hw, curColorGradient(1) * hw);
            float drdA = curColor - b0;

            pair->rJ.resF[idx] = residual*hw;

            pair->rJ.JIdx[0][idx] = hitColor[1];
            pair->rJ.JIdx[1][idx] = hitColor[2];
            pair->rJ.JabF[0][idx] = drdA*hw;
            pair->rJ.JabF[1][idx] = hw;

            JIdxJIdx_00+=hitColor[1]*hitColor[1];
            JIdxJIdx_11+=hitColor[2]*hitColor[2];
            JIdxJIdx_10+=hitColor[1]*hitColor[2];

            JabJIdx_00+= drdA*hw * hitColor[1];
            JabJIdx_01+= drdA*hw * hitColor[2];
            JabJIdx_10+= hw * hitColor[1];
            JabJIdx_11+= hw * hitColor[2];

            JabJab_00+= drdA*drdA*hw*hw;
            JabJab_01+= drdA*hw*hw;
            JabJab_11+= hw*hw;


            wJI2_sum += hw*hw*(hitColor[1]*hitColor[1]+hitColor[2]*hitColor[2]);

            if (!mOptimizeA.b()) {
                pair->rJ.JabF[0][idx]=0;
            }
            if (!mOptimizeB.b()) {
                pair->rJ.JabF[1][idx]=0;
            }

        }

    }

    pair->rJ.JIdx2(0,0) = JIdxJIdx_00;
    pair->rJ.JIdx2(0,1) = JIdxJIdx_10;
    pair->rJ.JIdx2(1,0) = JIdxJIdx_10;
    pair->rJ.JIdx2(1,1) = JIdxJIdx_11;
    pair->rJ.JabJIdx(0,0) = JabJIdx_00;
    pair->rJ.JabJIdx(0,1) = JabJIdx_01;
    pair->rJ.JabJIdx(1,0) = JabJIdx_10;
    pair->rJ.JabJIdx(1,1) = JabJIdx_11;
    pair->rJ.Jab2(0,0) = JabJab_00;
    pair->rJ.Jab2(0,1) = JabJab_01;
    pair->rJ.Jab2(1,0) = JabJab_01;
    pair->rJ.Jab2(1,1) = JabJab_11;

    if (!std::isfinite(energyLeft)) {
        pair->setNewState(DSORES_OOB);
        return pair->state_energy;
    }
    pair->state_NewEnergyWithOutlier = energyLeft;

    if(energyLeft > std::max<float>(hostData->frameEnergyTH, targetData->frameEnergyTH) || wJI2_sum < 2)
    {
        energyLeft = std::max<float>(hostData->frameEnergyTH, targetData->frameEnergyTH);
        pair->setNewState(DSORES_OUTLIER);
    }
    else
    {
        pair->setNewState(DSORES_IN);
    }

    pair->state_NewEnergy = energyLeft;
    return energyLeft;

}

int CML::Optimization::DSOBundleAdjustment::addToHessianTop(PPoint point, Ptr<DSOPoint, NonNullable> p, DSOResidualMode mode) {

    int nres = 0;

    Vector4f dc = mCDeltaF.cast<float>();

    float dd = p->deltaF;

    float bd_acc = 0;
    float Hdd_acc = 0;
    Vector4f Hcd_acc = Vector4f::Zero();

    for(auto r : p->getResiduals()) {

        if(mode == DSORES_ACTIVE)
        {
            if(r->isLinearized || !r->isActiveAndIsGoodNEW) continue;
        }
        if(mode == DSORES_LINEARIZED)
        {
            if(!r->isLinearized || !r->isActiveAndIsGoodNEW) continue;
        }
        if(mode == DSORES_MARGINALIZED)
        {
            if(!r->isActiveAndIsGoodNEW) continue;
            assert(r->isLinearized);
        }

        DSORawResidualJacobian &rJ = r->efsJ;
        int htIDX = unsafe_get(point->getReferenceFrame())->id + unsafe_get(r->elements.frame)->id * getFrames().size();
        Vector8f dp = mAdHTdeltaF[htIDX].cast<float>();


        List<dso::AccumulatorApprox> *acc = nullptr;


        Vector8 resApprox; // todo : pattern number
        if(mode == DSORES_ACTIVE) {
            resApprox = rJ.resF.cast<scalar_t>();
            acc = &mAccumulatorActive;
        }
        if(mode == DSORES_MARGINALIZED) {
            acc = &mAccumulatorActive; // We use the active accumulator for marginalized points
            resApprox = r->res_toZeroF.cast<scalar_t>();
        }
        if(mode == DSORES_LINEARIZED)
        {

            acc = &mAccumulatorLinearized;

            // compute Jp*delta
            __m128 Jp_delta_x = _mm_set1_ps(rJ.Jpdxi[0].dot(dp.head<6>())+rJ.Jpdc[0].dot(dc)+rJ.Jpdd[0]*dd);
            __m128 Jp_delta_y = _mm_set1_ps(rJ.Jpdxi[1].dot(dp.head<6>())+rJ.Jpdc[1].dot(dc)+rJ.Jpdd[1]*dd);
            __m128 delta_a = _mm_set1_ps((float)(dp[6]));
            __m128 delta_b = _mm_set1_ps((float)(dp[7]));

            for(int i=0;i<8;i+=4)
            {
                // PATTERN: rtz = resF - [JI*Jp Ja]*delta.
                __m128 rtz = _mm_load_ps(((float*)&r->res_toZeroF)+i);
                rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JIdx))+i),Jp_delta_x));
                rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JIdx+1))+i),Jp_delta_y));
                rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JabF))+i),delta_a));
                rtz = _mm_add_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JabF+1))+i),delta_b));
                _mm_store_ps(((float*)&resApprox)+i, rtz);
            }

        }


        // need to compute JI^T * r, and Jab^T * r. (both are 2-vectors).
        Vector2 JI_r(0,0);
        Vector2 Jab_r(0,0);
        float rr=0;
        for(int i=0;i<8;i++)
        {
            JI_r[0] += resApprox[i] *rJ.JIdx[0][i];
            JI_r[1] += resApprox[i] *rJ.JIdx[1][i];
            Jab_r[0] += resApprox[i] *rJ.JabF[0][i];
            Jab_r[1] += resApprox[i] *rJ.JabF[1][i];
            rr += resApprox[i]*resApprox[i];
        }

        (*acc)[htIDX].update(
                rJ.Jpdc[0].data(), rJ.Jpdxi[0].data(),
                rJ.Jpdc[1].data(), rJ.Jpdxi[1].data(),
                rJ.JIdx2(0,0),rJ.JIdx2(0,1),rJ.JIdx2(1,1));

        (*acc)[htIDX].updateBotRight(
                rJ.Jab2(0,0), rJ.Jab2(0,1), Jab_r[0],
                rJ.Jab2(1,1), Jab_r[1],rr);

        (*acc)[htIDX].updateTopRight(
                rJ.Jpdc[0].data(), rJ.Jpdxi[0].data(),
                rJ.Jpdc[1].data(), rJ.Jpdxi[1].data(),
                rJ.JabJIdx(0,0), rJ.JabJIdx(0,1),
                rJ.JabJIdx(1,0), rJ.JabJIdx(1,1),
                JI_r[0], JI_r[1]);

        Vector2f Ji2_Jpdd = rJ.JIdx2 * rJ.Jpdd;
        bd_acc +=  JI_r[0]*rJ.Jpdd[0] + JI_r[1]*rJ.Jpdd[1];
        Hdd_acc += Ji2_Jpdd.dot(rJ.Jpdd);
        Hcd_acc += rJ.Jpdc[0] * Ji2_Jpdd[0] + rJ.Jpdc[1] * Ji2_Jpdd[1];

        nres++;
    }

    if(mode == DSORES_ACTIVE)
    {
        p->Hdd_accAF = Hdd_acc;
        p->bd_accAF = bd_acc;
        p->Hcd_accAF = Hcd_acc;
        p->haveAF = true;
    }
    if(mode == DSORES_LINEARIZED || mode == DSORES_MARGINALIZED)
    {
        p->Hdd_accLF = Hdd_acc;
        p->bd_accLF = bd_acc;
        p->Hcd_accLF = Hcd_acc;
        p->haveLF = true;
    }
    if(mode == DSORES_MARGINALIZED)
    {
        p->Hcd_accAF.setZero();
        p->Hdd_accAF = 0;
        p->bd_accAF = 0;
        p->haveAF = false;
    }

    return nres;

}

void CML::Optimization::DSOBundleAdjustment::stitchDoubleTop(List<dso::AccumulatorApprox> &acc, Matrix<Dynamic, Dynamic> &fH, Vector<Dynamic> &fb, bool usePrior) {

    #pragma omp single
    {
#if CML_USE_OPENMP
        int ompNumThread = omp_get_num_threads();
#else
        int ompNumThread = 1;
#endif
        if (fH.rows() != getFrames().size() * 8 + 4) {
            fH = Matrix<Dynamic, Dynamic>::Zero(getFrames().size() * 8 + 4, getFrames().size() * 8 + 4);
            fb = Vector<Dynamic>::Zero(getFrames().size() * 8 + 4);
        } else {
            fH.setZero();
            fb.setZero();
        }
        if (sdt_tH.size() != ompNumThread) {
            sdt_tH.resize(ompNumThread);
            sdt_tb.resize(ompNumThread);
            for (int i = 0; i < sdt_tH.size(); i++) {
                sdt_tH[i] = Matrix<Dynamic, Dynamic>::Zero(getFrames().size()*8+4, getFrames().size()*8+4);
                sdt_tb[i] = Vector<Dynamic>::Zero(getFrames().size()*8+4);
            }
        }
        if (sdt_tH[0].rows() != getFrames().size() * 8 + 4) {
            for (int i = 0; i < sdt_tH.size(); i++) {
                sdt_tH[i] = Matrix<Dynamic, Dynamic>::Zero(getFrames().size()*8+4, getFrames().size()*8+4);
                sdt_tb[i] = Vector<Dynamic>::Zero(getFrames().size()*8+4);
            }
        }
    }

    #if CML_USE_OPENMP
    #pragma omp for
    #endif
    for(size_t h=0;h<getFrames().size();h++) {
        for (size_t t = 0; t < getFrames().size(); t++) {

#if CML_USE_OPENMP
            int tid = omp_get_thread_num();
#else
            int tid = 0;
#endif

            Matrix<Dynamic, Dynamic> &tH = sdt_tH[tid]; // Matrix<Dynamic, Dynamic>::Zero(getFrames().size()*8+4, getFrames().size()*8+4);
            Vector<Dynamic> &tb = sdt_tb[tid]; //Vector<Dynamic>::Zero(getFrames().size()*8+4);

            tH.setZero();
            tb.setZero();

            int hIdx = 4 + h * 8;
            int tIdx = 4 + t * 8;
            int aidx = h + getFrames().size() * t;


            acc[aidx].finish();
            if (acc[aidx].num == 0) continue;

            Matrix<8 + 4 + 1, 8 + 4 + 1> accH = acc[aidx].H.cast<scalar_t>();


            tH.block<8, 8>(hIdx, hIdx).noalias() += mAdHost[aidx] * accH.block<8, 8>(4, 4) * mAdHost[aidx].transpose();

            tH.block<8, 8>(tIdx, tIdx).noalias() += mAdTarget[aidx] * accH.block<8, 8>(4, 4) * mAdTarget[aidx].transpose();

            tH.block<8, 8>(hIdx, tIdx).noalias() += mAdHost[aidx] * accH.block<8, 8>(4, 4) * mAdTarget[aidx].transpose();

            tH.block<8, 4>(hIdx, 0).noalias() += mAdHost[aidx] * accH.block<8, 4>(4, 0);

            tH.block<8, 4>(tIdx, 0).noalias() += mAdTarget[aidx] * accH.block<8, 4>(4, 0);

            tH.topLeftCorner<4, 4>().noalias() += accH.block<4, 4>(0, 0);

            tb.segment<8>(hIdx).noalias() += mAdHost[aidx] * accH.block<8, 1>(4, 8 + 4);

            tb.segment<8>(tIdx).noalias() += mAdTarget[aidx] * accH.block<8, 1>(4, 8 + 4);

            tb.head<4>().noalias() += accH.block<4, 1>(0, 8 + 4);

            #pragma omp critical
            {
                fH += tH;
                fb += tb;
            }



        }
    }

#pragma omp single
    {
        if (usePrior) {
            fH.diagonal().head<4>() += mCPrior;
            fb.head<4>() += mCPrior.cwiseProduct(mCDeltaF.cast<scalar_t>());
            for (size_t h = 0; h < getFrames().size(); h++) {
                auto self = get(getFrames()[h]);
                fH.diagonal().segment<8>(4 + h * 8) += self->prior;
                fb.segment<8>(4 + h * 8) += self->prior.cwiseProduct(self->delta_prior);
            }
        }

        for (size_t h = 0; h < getFrames().size(); h++) {
            int hIdx = 4 + h * 8;
            fH.block<4, 8>(0, hIdx).noalias() = fH.block<8, 4>(hIdx, 0).transpose();

            for (size_t t = h + 1; t < getFrames().size(); t++) {
                int tIdx = 4 + t * 8;
                fH.block<8, 8>(hIdx, tIdx).noalias() += fH.block<8, 8>(tIdx, hIdx).transpose();
                fH.block<8, 8>(tIdx, hIdx).noalias() = fH.block<8, 8>(hIdx, tIdx).transpose();
            }
        }
    }
}

void CML::Optimization::DSOBundleAdjustment::addToHessianSC(PPoint point, Ptr<DSOPoint, NonNullable> self, bool shiftPriorToZero) {
    auto selfHost = get(point->getReferenceFrame());

    int ngoodres = 0;
    for(auto r : self->getResiduals()) if(r->isActiveAndIsGoodNEW) ngoodres++;
    if(ngoodres==0)
    {
        self->HdiF=0;
        self->bdSumF=0;
        self->setInverseDepthHessian(0);
        self->setMaxRelBaseline(0);
        self->updatePointUncertainty(point, mScaledVarTH.f(), mAbsVarTH.f(), mMinRelBS.f());
        return;
    }

    float H = self->Hdd_accAF + self->Hdd_accLF + self->priorF;
    if(H < 1e-10) H = 1e-10;

    self->setInverseDepthHessian(H);
    self->updatePointUncertainty(point, mScaledVarTH.f(), mAbsVarTH.f(), mMinRelBS.f());

    self->HdiF = 1.0 / H;
    self->bdSumF = self->bd_accAF + self->bd_accLF;
    if(shiftPriorToZero) self->bdSumF += self->priorF*self->deltaF;
    Vector4f Hcd = self->Hcd_accAF + self->Hcd_accLF;
    mAccHcc.update(Hcd,Hcd,self->HdiF);
    mAccbc.update(Hcd, self->bdSumF * self->HdiF);

    assert(std::isfinite((float)(self->HdiF)));

    int nFrames2 = getFrames().size() * getFrames().size();
    for(auto r1 : self->getResiduals())
    {
        if(!r1->isActiveAndIsGoodNEW) continue;

        auto selfTarget1 = get(r1->elements.frame);


        int r1ht = selfHost->id + selfTarget1->id * getFrames().size(); // todo : check r1 / r2

        for(auto r2 : self->getResiduals())
        {
            if(!r2->isActiveAndIsGoodNEW) continue;

            auto selfTarget2 = get(r2->elements.frame);

            mAccD[r1ht+selfTarget2->id*nFrames2].update(r1->JpJdF, r2->JpJdF, self->HdiF);
        }

        mAccE[r1ht].update(r1->JpJdF, Hcd, self->HdiF);
        mAccEB[r1ht].update(r1->JpJdF,self->HdiF*self->bdSumF);
    }
}

void CML::Optimization::DSOBundleAdjustment::stitchDoubleSC(Matrix<Dynamic, Dynamic> &fH, Vector<Dynamic> &fb) {
    int nframes2 = getFrames().size() *getFrames().size();

    fH = Matrix<Dynamic, Dynamic>::Zero(getFrames().size()*8+4, getFrames().size()*8+4);
    fb = Vector<Dynamic>::Zero(getFrames().size()*8+4);

    Mutex mutex;

    #if CML_USE_OPENMP
    #pragma omp for collapse(2) schedule(static) ordered
    #endif
    for(size_t i=0;i<getFrames().size();i++) {
        for (size_t j = 0; j < getFrames().size(); j++) {

            Matrix<Dynamic, Dynamic> tH = Matrix<Dynamic, Dynamic>::Zero(getFrames().size()*8+4, getFrames().size()*8+4);
            Vector<Dynamic> tb = Vector<Dynamic>::Zero(getFrames().size()*8+4);

            int iIdx = 4 + i * 8;
            int jIdx = 4 + j * 8;
            int ijIdx = i + getFrames().size() * j;

            mAccE[ijIdx].finish();
            mAccEB[ijIdx].finish();

            Matrix<8, 4> accEM = mAccE[ijIdx].A1m.cast<scalar_t>();
            Vector<8> accEBV = mAccEB[ijIdx].A1m.cast<scalar_t>();

            tH.block<8, 4>(iIdx, 0) += mAdHost[ijIdx] * accEM;
            tH.block<8, 4>(jIdx, 0) += mAdTarget[ijIdx] * accEM;

            fb.segment<8>(iIdx) += mAdHost[ijIdx] * accEBV;
            fb.segment<8>(jIdx) += mAdTarget[ijIdx] * accEBV;

            for (size_t k = 0; k < getFrames().size(); k++) {
                int kIdx = 4 + k * 8;
                int ijkIdx = ijIdx + k * nframes2;
                int ikIdx = i + getFrames().size() * k;

                mAccD[ijkIdx].finish();
                if (mAccD[ijkIdx].num == 0) continue;
                Matrix<8, 8> accDM = mAccD[ijkIdx].A1m.cast<scalar_t>();

                tH.block<8, 8>(iIdx, iIdx) += mAdHost[ijIdx] * accDM * mAdHost[ikIdx].transpose();

                tH.block<8, 8>(jIdx, kIdx) += mAdTarget[ijIdx] * accDM * mAdTarget[ikIdx].transpose();

                tH.block<8, 8>(jIdx, iIdx) += mAdTarget[ijIdx] * accDM * mAdHost[ikIdx].transpose();

                tH.block<8, 8>(iIdx, kIdx) += mAdHost[ijIdx] * accDM * mAdTarget[ikIdx].transpose();

            }

            LockGuard lg(mutex);
            fH += tH;
            fb += tb;

        }
    }

    mAccHcc.finish();
    mAccbc.finish();
    fH.topLeftCorner<4,4>() = mAccHcc.A1m.cast<scalar_t>();
    fb.head<4>() = mAccbc.A1m.cast<scalar_t>();

    // ----- new: copy transposed parts for calibration only.
    for(size_t h=0; h < getFrames().size(); h++)
    {
        int hIdx = 4+h*8;
        fH.block<4,8>(0,hIdx).noalias() = fH.block<8,4>(hIdx,0).transpose();

        if (!std::isfinite(fH.norm()) || !std::isfinite(fb.norm())) {
            abort();
        }

    }
}

void CML::Optimization::DSOBundleAdjustment::applyActiveRes(bool copyJacobians) {
    for (auto r : mActiveResiduals) {
        applyRes(r, copyJacobians);
    }
}

void CML::Optimization::DSOBundleAdjustment::applyRes(DSOResidual* residual, bool copyJacobians) {

    if(copyJacobians)
    {
        if(residual->getState() == DSORES_OOB)
        {
            assert(!residual->isActiveAndIsGoodNEW);
            return;	// can never go back from OOB
        }
        if(residual->getNewState() == DSORES_IN)// && )
        {
            residual->isActiveAndIsGoodNEW = true;

            std::swap(residual->rJ, residual->efsJ);

            Vector2f JI_JI_Jd = residual->efsJ.JIdx2 * residual->efsJ.Jpdd;

            for(int i=0;i<6;i++) {
                residual->JpJdF[i] = residual->efsJ.Jpdxi[0][i] * JI_JI_Jd[0] + residual->efsJ.Jpdxi[1][i] * JI_JI_Jd[1];
            }

            residual->JpJdF.segment<2>(6) = residual->efsJ.JabJIdx * residual->efsJ.Jpdd;

        }
        else
        {
            residual->isActiveAndIsGoodNEW=false;
        }
    }

    residual->applyNewState();
    residual->state_energy = residual->state_NewEnergy;
}

CML::scalar_t CML::Optimization::DSOBundleAdjustment::calcMEnergy() {
    assert(mDeltaValid);
    assert(mAdjointsValid);
    assert(mIndicesValid);

    if (mForceAccept.b()) {
        return 0;
    }

    Vector<Dynamic> delta = Vector<Dynamic>(4 + getFrames().size() * 8);
    delta.head<4>() = mCDeltaF.cast<scalar_t>();
    for(size_t h = 0; h < getFrames().size(); h++) {
        delta.segment<8>(4 + 8 * h) = get(getFrames()[h])->delta.cast<scalar_t>();
    }

    //std::cout << "M Energy : " << delta.dot(2 * mMarginalizedB + mMarginalizedHessian * delta) << std::endl;

    scalar_t energy = abs(delta.dot(2 * mMarginalizedB + mMarginalizedHessian * delta.cast<scalar_t>()));

    return energy;
}

CML::scalar_t CML::Optimization::DSOBundleAdjustment::calcLEnergy() {

    assert(mDeltaValid);
    assert(mAdjointsValid);
    assert(mIndicesValid);

    if (mForceAccept.b()) {
        return 0;
    }

    int num = 0;

    double F = 0;
    for(PFrame frame : getFrames()) {
        auto frameData = get(frame);
        F += frameData->delta_prior.cwiseProduct(frameData->prior).dot(frameData->delta_prior);
    }

    // todo
    float setting_initialCalibHessian = 5e9;
    mCPrior = Vector4::Constant(setting_initialCalibHessian);

    F += mCDeltaF.cwiseProduct(Vector4::Constant(5e9)).dot(mCDeltaF);


    dso::Accumulator11 E;
    E.initialize();
    Vector4f dc = mCDeltaF.cast<float>();

    for(auto point : getPoints())
    {
        auto p = get(point);
        auto selfHost = get(point->getReferenceFrame());

        float dd = p->deltaF;

        for(auto r : p->getResiduals())
        {
            if(!r->isLinearized || !r->isActiveAndIsGoodNEW) continue;
            num++;

            auto selfTarget = get(r->elements.frame);

            Matrixf<1, 8> dp = mAdHTdeltaF[selfHost->id + getFrames().size() * selfTarget->id].cast<float>();
            DSORawResidualJacobian& rJ = r->efsJ;



            // compute Jp*delta
            float Jp_delta_x_1 =  rJ.Jpdxi[0].dot(dp.head<6>())
                                  +rJ.Jpdc[0].dot(dc)
                                  +rJ.Jpdd[0]*dd;

            float Jp_delta_y_1 =  rJ.Jpdxi[1].dot(dp.head<6>())
                                  +rJ.Jpdc[1].dot(dc)
                                  +rJ.Jpdd[1]*dd;

            __m128 Jp_delta_x = _mm_set1_ps(Jp_delta_x_1);
            __m128 Jp_delta_y = _mm_set1_ps(Jp_delta_y_1);
            __m128 delta_a = _mm_set1_ps((float)(dp[6]));
            __m128 delta_b = _mm_set1_ps((float)(dp[7]));

            for(int i=0;i+3<8;i+=4) // todo : pattern num
            {
                // PATTERN: E = (2*res_toZeroF + J*delta) * J*delta.
                __m128 Jdelta =            _mm_mul_ps(_mm_load_ps(((float*)(rJ.JIdx))+i),Jp_delta_x);
                Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JIdx+1))+i),Jp_delta_y));
                Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JabF))+i),delta_a));
                Jdelta = _mm_add_ps(Jdelta,_mm_mul_ps(_mm_load_ps(((float*)(rJ.JabF+1))+i),delta_b));

                __m128 r0 = _mm_load_ps(((float*)&r->res_toZeroF)+i);
                r0 = _mm_add_ps(r0,r0);
                r0 = _mm_add_ps(r0,Jdelta);
                Jdelta = _mm_mul_ps(Jdelta,r0);
                E.updateSSENoShift(Jdelta);
            }
            for(int i=((8>>2)<<2); i < 8; i++) // todo : pattern num two times
            {
                float Jdelta = rJ.JIdx[0][i]*Jp_delta_x_1 + rJ.JIdx[1][i]*Jp_delta_y_1 +
                               rJ.JabF[0][i]*dp[6] + rJ.JabF[1][i]*dp[7];
                E.updateSingleNoShift((float)(Jdelta * (Jdelta + 2 * r->res_toZeroF[i])));
            }
        }
        E.updateSingle(p->deltaF * p->deltaF * p->priorF);
    }
    E.finish();

    mStatisticNumLinearized->addValue(num);

    return E.A + F;

}

void CML::Optimization::DSOBundleAdjustment::fixLinearization(DSOResidual* residual) {
    auto selfHost = get(residual->elements.mapPoint->getReferenceFrame());
    auto selfTarget = get(residual->elements.frame);
    auto selfPoint = get(residual->elements.mapPoint);

    auto &J = residual->efsJ;

    Vector8f dp = mAdHTdeltaF[ selfHost->id + getFrames().size() * selfTarget->id ].cast<float>();

    // compute Jp*delta
    __m128 Jp_delta_x = _mm_set1_ps(J.Jpdxi[0].dot(dp.head<6>()) + J.Jpdc[0].dot(mCDeltaF.cast<float>()) + J.Jpdd[0] * selfPoint->deltaF);
    __m128 Jp_delta_y = _mm_set1_ps(J.Jpdxi[1].dot(dp.head<6>()) + J.Jpdc[1].dot(mCDeltaF.cast<float>()) + J.Jpdd[1] * selfPoint->deltaF);
    __m128 delta_a = _mm_set1_ps((float)(dp[6]));
    __m128 delta_b = _mm_set1_ps((float)(dp[7]));

    for(int i = 0; i < 8; i+= 4) // todo : Pattern num
    {
        // PATTERN: rtz = resF - [JI*Jp Ja]*delta.
        __m128 rtz = _mm_load_ps(((float*)&J.resF)+i);

        rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)J.JIdx[0].data())+i),Jp_delta_x));
        rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)J.JIdx[1].data())+i),Jp_delta_y));
        rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)J.JabF[0].data())+i),delta_a));
        rtz = _mm_sub_ps(rtz,_mm_mul_ps(_mm_load_ps(((float*)J.JabF[1].data())+i),delta_b));
        _mm_store_ps(((float*)&residual->res_toZeroF)+i, rtz);
    }

    residual->isLinearized = true;
}

void CML::Optimization::DSOBundleAdjustment::tryMarginalize() {

    assert(mIndicesValid);

    List<PFrame> fhsToKeepPoints;
    List<PFrame> fhsToMargPoints;

    // todo
    int setting_minGoodActiveResForMarg=3;
    int setting_minGoodResForMarg=4;



    for(int i=((int)getFrames().size())-1;i>=0 && i >= ((int)getFrames().size());i--) {
        auto frameData = get(getFrames()[i]);
        if (!frameData->flaggedForMarginalization) {
            fhsToKeepPoints.push_back(getFrames()[i]);
        }
    }

    for(int i=0; i< (int)getFrames().size();i++) {
        auto frameData = get(getFrames()[i]);
        if (frameData->flaggedForMarginalization) {
            fhsToMargPoints.push_back(getFrames()[i]);
        }
    }


    int flag_oob=0, flag_in=0, flag_inin=0, flag_nores=0, flag_flagformarg=0;


    List<PPoint> pointToDrop, pointToMarginalize;

    for (auto point : getPoints()) {

        auto ph = get(point);
        auto host = get(point->getReferenceFrame());


        if(point->getReferenceInverseDepth() < 0 || ph->getResiduals().empty())
        {
            pointToDrop.emplace_back(point);
            flag_oob++;
        }
        else if(isOOB(point, fhsToKeepPoints, fhsToMargPoints) || host->flaggedForMarginalization)
        {
            if((int)ph->getResiduals().size() >= setting_minGoodActiveResForMarg && ph->numGoodResiduals >= setting_minGoodResForMarg)
            {
                flag_in++;
                if (host->flaggedForMarginalization) flag_flagformarg++;
                int ngoodRes=0;
                for(auto r : ph->getResiduals())
                {
                    auto hostData = get(r->elements.mapPoint->getReferenceFrame());
                    auto targetData =  get(r->elements.frame);

                    r->resetOOB();
                    linearize(r, DSOFramePrecomputed(hostData.p(), targetData.p()));
                    r->isLinearized = false;
                    applyRes(r, true);
                    if(r->isActiveAndIsGoodNEW)
                    {
                        fixLinearization(r);
                        ngoodRes++;
                    }
                }
                /*if(point->getUncertainty() < 1.0 / mMinIdepthHMarg.f())
                {
                    pointToMarginalize.emplace_back(point);
                    flag_inin++;
                }
                else
                {
                    pointToDrop.emplace_back(point);
                    flag_oob++;
                }*/
                if(ph->getInverseDepthHessian() > mMinIdepthHMarg.f())
                {
                    pointToMarginalize.emplace_back(point);
                    flag_inin++;
                }
                else
                {
                    pointToDrop.emplace_back(point);
                    flag_oob++;
                }

            }
            else
            {
                pointToDrop.emplace_back(point);
                flag_oob++;
            }

        }

    }

    logger.info("DSO BA is using actually " + std::to_string(getPoints().size()) + " points");
    logger.info("DSO BA is dropping " + std::to_string(pointToDrop.size()) + " points");
    logger.info("DSO BA will marginalize " + std::to_string(pointToMarginalize.size()) + " more points");


    for (auto point : pointToDrop) {
        removePoint(point);
        mOutliers.insert(point);
        free(point);
    }

    for (auto point : pointToMarginalize) {
        point->setGroup(DSOTOMARGINALIZE, true);
    }

    //int remainingPoints = getPoints().size() - getMap().getGroupMapPoints(DSOTOMARGINALIZE).size();
    //assertThrow(remainingPoints > 50, "If we continue the marginalization, " + std::to_string(remainingPoints) + " will remain... We added " + std::to_string(pointToMarginalize.size()) + " points to marginalize. The BA is using" + std::to_string( getPoints().size()) + " points." );

    mStatisticOOB->addValue((float)flag_oob);
    mStatisticIn->addValue((float)flag_in);
    mStatisticInIn->addValue((float)flag_inin);
    mStatisticNores->addValue((float)flag_nores);



}

void CML::Optimization::DSOBundleAdjustment::computeNullspaces() {
    mLastNullspaces_pose.clear();
    mLastNullspaces_scale.clear();
    mLastNullspaces_affA.clear();
    mLastNullspaces_affB.clear();


    int n = getFrames().size() * 8 + 4;
    List<Vector<Dynamic>> nullspaces_x0_pre;
    for(int i = 0; i < 6; i++)
    {
        Vector<Dynamic> nullspace_x0(n);
        nullspace_x0.setZero();
        for(PFrame frame : getFrames())
        {
            auto fh = get(frame);
            nullspace_x0.segment<6>(CPARS + fh->id * 8) = fh->nullspaces_pose.col(i);
            nullspace_x0.segment<3>(CPARS + fh->id * 8) *= 1.0 / mScaleTranslation.f();
            nullspace_x0.segment<3>(CPARS + fh->id * 8+3) *= 1.0 / mScaleRotation.f();
        }
        nullspaces_x0_pre.push_back(nullspace_x0);
        mLastNullspaces_pose.push_back(nullspace_x0);
    }
    for(int i=0;i<2;i++)
    {
        Vector<Dynamic> nullspace_x0(n);
        nullspace_x0.setZero();
        for(PFrame frame : getFrames())
        {
            auto fh = get(frame);
            nullspace_x0.segment<2>(CPARS+fh->id*8+6) = fh->nullspaces_affine.col(i).head<2>();
            nullspace_x0[CPARS+fh->id*8+6] *= 1.0 / mScaleLightA.f();
            nullspace_x0[CPARS+fh->id*8+7] *= 1.0 / mScaleLightB.f();
        }
        nullspaces_x0_pre.push_back(nullspace_x0);
        if(i==0) mLastNullspaces_affA.push_back(nullspace_x0);
        if(i==1) mLastNullspaces_affB.push_back(nullspace_x0);
    }

    Vector<Dynamic> nullspace_x0(n);
    nullspace_x0.setZero();
    for(PFrame frame : getFrames())
    {
        auto fh = get(frame);
        nullspace_x0.segment<6>(CPARS+fh->id*8) = fh->nullspaces_scale;
        nullspace_x0.segment<3>(CPARS+fh->id*8) *= 1.0 / mScaleTranslation.f();
        nullspace_x0.segment<3>(CPARS+fh->id*8+3) *= 1.0 / mScaleRotation.f();
    }
    nullspaces_x0_pre.push_back(nullspace_x0);
    mLastNullspaces_scale.push_back(nullspace_x0);


}

void CML::Optimization::DSOBundleAdjustment::setNewFrameEnergyTH() {
    // collect all residuals and make decision on TH.
    List<float> allResVec;
    allResVec.reserve(mActiveResiduals.size() * 2);
    PFrame newFrame = getFrames().back();
    auto newFrameData = get(newFrame);

    for(auto r : mActiveResiduals) {
        if (r->state_NewEnergyWithOutlier >= 0 && r->elements.frame == newFrame) {
            allResVec.emplace_back(r->state_NewEnergyWithOutlier);
        }
    }

    if(allResVec.empty())
    {
        newFrameData->frameEnergyTH = 12 * 12 * 8; // todo : here, 8 is the pattern number
        return;		// should never happen, but lets make sure.
    }

// TODO OOOOOOOOOO

    float setting_frameEnergyTHConstWeight = 0.5;
    float setting_frameEnergyTHN = 0.7f;
    float setting_frameEnergyTHFacMedian = 1.5;
    float setting_overallEnergyTHWeight = 1;

    // TODO OOOOOOOOO


    int nthIdx = setting_frameEnergyTHN * allResVec.size();

    assert(nthIdx < (int)allResVec.size());
    assert(setting_frameEnergyTHN < 1);

    // std::sort(allResVec.begin(), allResVec.end());
    std::nth_element(allResVec.begin(), allResVec.begin()+nthIdx, allResVec.end());
    float nthElement = sqrtf(allResVec[nthIdx]);


    newFrameData->frameEnergyTH = nthElement * setting_frameEnergyTHFacMedian;
    newFrameData->frameEnergyTH = 26.0f * setting_frameEnergyTHConstWeight + newFrameData->frameEnergyTH * ( 1 - setting_frameEnergyTHConstWeight);
    newFrameData->frameEnergyTH = newFrameData->frameEnergyTH * newFrameData->frameEnergyTH;
    newFrameData->frameEnergyTH *= setting_overallEnergyTHWeight * setting_overallEnergyTHWeight;


}

void CML::Optimization::DSOBundleAdjustment::marginalizePointsF()
{



    assert(mDeltaValid);
    assert(mAdjointsValid);
    assert(mIndicesValid);

    computeDelta();
    setZero();

    Set<PPoint, Hasher> allPointsToMarg = getMap().getGroupMapPoints(DSOTOMARGINALIZE);

    if (allPointsToMarg.size() > 0) {
        logger.info("DSO Bundle Adjustment marginalize " + std::to_string(allPointsToMarg.size()) + " points");
    }

    int nres = 0;
    computeAdjoints();
    for(auto point : allPointsToMarg)
    {
        nres += addToHessianTop(point, get(point), DSORES_MARGINALIZED);
        addToHessianSC(point, get(point), false);
        point->setMarginalized(true);
        point->setGroup(DSOMARGINALIZED, true);
        point->setGroup(DSOTOMARGINALIZE, false);
        removePoint(point, true);
    }

    Matrix<Dynamic, Dynamic> M, Msc;
    Vector<Dynamic> Mb, Mbsc;

    stitchDoubleTop(mAccumulatorActive, M, Mb, false);
    stitchDoubleSC(Msc, Mbsc);

    Matrix<Dynamic, Dynamic> H =  M-Msc;
    Vector<Dynamic> b =  Mb-Mbsc;

    scalar_t setting_margWeightFac = 0.5*0.5; // todo;

    mMarginalizedHessian += setting_margWeightFac * H;
    mMarginalizedB += setting_margWeightFac * b;

    mIndicesValid = false;
    makeIDX();

}

bool CML::Optimization::DSOBundleAdjustment::isOOB(PPoint p,
                                                        const List<PFrame> &toKeep,
                                                        const List<PFrame> &toMarg) {


    int   setting_minGoodActiveResForMarg=3;
    int   setting_minGoodResForMarg=4;
    // todo

    auto pointData = get(p);

    int visInToMarg = 0, numIn = 0;
    for (auto r : pointData->getResiduals()) {
        if (r->getState() != DSORES_IN) {
            continue;
        }
        numIn++;
        for (auto k : toMarg) {
            if (r->elements.frame == k) visInToMarg++;
        }
    }

    if(numIn >= setting_minGoodActiveResForMarg &&
       pointData->numGoodResiduals > setting_minGoodResForMarg+10 &&
       (int)numIn - visInToMarg < setting_minGoodActiveResForMarg) {
        return true;
    }




    if(pointData->getLastResidual(0).second == DSORES_OOB) return true;
    if(numIn < 2) {
        return false;
    }
    if(pointData->getLastResidual(0).second == DSORES_OUTLIER && pointData->getLastResidual(1).second == DSORES_OUTLIER) {
        return true;
    }
    return false;
}

void CML::Optimization::DSOBundleAdjustment::makeIDX() {
    makeFrameId();
    mIndicesValid = true;
}

void CML::Optimization::DSOBundleAdjustment::onValueChange(const Parameter &parameter) {
    if (parameter == mScaledVarTH || parameter == mAbsVarTH || parameter == mMinRelBS) {
        for (auto point : getPoints()) {
            get(point)->updatePointUncertainty(point, mScaledVarTH.f(), mAbsVarTH.f(),
                                                        mMinRelBS.f());
        }
        for (auto point : getMap().getGroupMapPoints(DSOMARGINALIZED)) {
            get(point)->updatePointUncertainty(point, mScaledVarTH.f(), mAbsVarTH.f(),
                                                        mMinRelBS.f());
        }
    }
}

void CML::Optimization::DSOBundleAdjustment::addIndirectToProblem(Vector<Dynamic> &X) {
    if (!mMixedBundleAdjustment.b()) {
        return;
    }

    List<PFrame> frames = getFrames();
    mIndirectPointToOptimizeSet.clear();
    for (int i = 0; i < frames.size(); i++) {
        PFrame frame = frames[i];
        frame->getGroupMapPointsNoClean(getMap().INDIRECTGROUP, mIndirectPointToOptimizeSet);
    }
    List<PPoint> pointsToOptimize(mIndirectPointToOptimizeSet.begin(), mIndirectPointToOptimizeSet.end());

    if (pointsToOptimize.size() == 0) {
        return;
    }

    int Hsize = getFrames().size()*6 + pointsToOptimize.size() * 3;
    int pstart = getFrames().size()*6;

    List<int> numPointsPerFrame;
    numPointsPerFrame.resize(getFrames().size(), 0);
    Vector<Dynamic> b = Vector<Dynamic>::Zero(Hsize);
    Matrix<Dynamic, Dynamic> H = Matrix<Dynamic, Dynamic>::Zero(Hsize, Hsize);
    Eigen::SparseMatrix<scalar_t> J(Hsize, getFrames().size() * pointsToOptimize.size());

    // todo : for the moment we are not using the schur complement, so include pointsToOptimize in H

    List<Vector3> Jpoints;
    Jpoints.resize(pointsToOptimize.size(), Vector3::Zero());

    // todo : use ceres autograd here

    for (int i = 0; i < frames.size(); i++) {
        PFrame frame = frames[i];
        auto frameData = get(frame);

        for (int j = 0; j < pointsToOptimize.size(); j++) {
            PPoint point = pointsToOptimize[j];
            auto fp = frame->getFeaturePoint(point);
            if (!fp.has_value()) {
                continue;
            }
            Vector3 currentCoordinate = point->getWorldCoordinate().absolute();


            // todo : warning, + operator are not the same for se(3) space and SE(3) space
            SE3 expValue = Sophus::SE3<scalar_t>(frame->getCamera().getRotationMatrix(), frame->getCamera().getTranslation()); //  todo : get state scaler ??? waiiit
            Vector6 currentCamera = expValue.log();
            Matrix<7,6> expDerivative = Sophus::SE3<scalar_t>::Dx_exp_x(currentCamera);
            Vector7 cameraDerivative;

            scalar_t currentResidual = 0;
            Vector3 pointJacobian;
            bool res = ReprojectionError(frame, point).jacobian(currentResidual, frame->getCamera(), currentCoordinate, cameraDerivative, pointJacobian);

            if (!res || currentResidual > 4 * 4) {
                //std::cout << "error" << std::endl;
                continue;
            }


            Jpoints[j] += pointJacobian;

            //currentResidual = currentResidual / fp.value().processScaleFactorFromLevel();


            Matrix<Dynamic, Dynamic> finalDerivativeM = (cameraDerivative.transpose() * expDerivative);

            Vector6 finalDerivative = finalDerivativeM.transpose();

            //finalDerivative.head<3>() *= mScaleTranslation.f();
            //finalDerivative.tail<3>() *= mScaleRotation.f();
            for (int k = 0; k < 6; k++) {
                J.insert(frameData->id * 6 + k, frameData->id * pointsToOptimize.size() + j) = finalDerivative(k);
            }
            for (int k = 0; k < 3; k++) {
                J.insert(pstart + j * 3 + k,frameData->id * pointsToOptimize.size() + j) = pointJacobian(k);
            }
            //J.block(frameData->id * 6,frameData->id * pointsToOptimize.size() + j,6,1) = finalDerivative;
            //J.block(pstart + j * 3,frameData->id * pointsToOptimize.size() + j,3,1) = pointJacobian;
            b.segment<6>(frameData->id * 6) += finalDerivative * currentResidual;
            b.segment<3>(pstart + j * 3) += pointJacobian * currentResidual;
            numPointsPerFrame[frameData->id]++;
        }
    }


    // todo : how to do this fast
    J.makeCompressed();
    H = J * J.transpose();

#define INDIRECT_MARGINALIZATION 0

#if INDIRECT_MARGINALIZATION
    Matrix<Dynamic, Dynamic> H11 = H.block(0,0,getFrames().size()*6,getFrames().size()*6);
    Matrix<Dynamic, Dynamic> H21 = H.block(pstart,0,pointsToOptimize.size()*3,getFrames().size()*6);
    Matrix<Dynamic, Dynamic> H12 = H.block(0,pstart,getFrames().size()*6,pointsToOptimize.size()*3);
    Matrix<Dynamic, Dynamic> H22 = H.block(pstart,pstart,pointsToOptimize.size()*3,pointsToOptimize.size()*3);

    Matrix<Dynamic, Dynamic> H22inv = H22.inverse(); // todo : inverse by block of 3x3 instead
    // Matrix<Dynamic, Dynamic> H22inv = H22.llt().solve(Matrix<Dynamic, Dynamic>::Identity(pointsToOptimize.size()*3,pointsToOptimize.size()*3));

    Vector<Dynamic> b1 = b.segment(0,getFrames().size()*6);
    Vector<Dynamic> b2 = b.segment(pstart,pointsToOptimize.size()*3);

    // todo : i think they might be a problem with the matrix size. run this in debug pleasssse
    Matrix<Dynamic, Dynamic> M = H11 - H12 * H22inv * H21;
    Vector<Dynamic> bM = b1 - H12 * H22inv * b2;
#else

    Matrix<Dynamic, Dynamic> M = H.block(0,0,getFrames().size()*6,getFrames().size()*6);
    Vector<Dynamic> bM = b.segment(0,getFrames().size()*6);
#endif
    //H = H.block(0,0,getFrames().size()*6,getFrames().size()*6);

    for (int j = 0; j < pointsToOptimize.size(); j++) {
        pointsToOptimize[j]->setUncertainty((Jpoints[j] * Jpoints[j].transpose()).inverse().diagonal().norm());
    }
    // todo : use dso accumulator here

    for(size_t i = 0; i < M.rows(); i++) {
        M(i,i) *= (1+mFixedLambda.f());
    }

    // todo : i think there is a minus here
    Vector<Dynamic> indirectX = M.ldlt().solve(-bM);

    if (!indirectX.allFinite()) {
        return;
    }

    List<int> numDirectPointsPerFrame;
    numDirectPointsPerFrame.resize(getFrames().size(), 0);

    for (auto frame : getFrames()) {
        auto frameData = get(frame);
        numDirectPointsPerFrame[frameData->id] += frameData->getPoints().size(); // todo : take in account outliers
    }

    // todo : how to weight this
    // todo : by the number of points (for dso multiply by 8)
    // todo : but ! we have different number of points for each frames
    for(size_t i = 0; i < getFrames().size(); i++) {
        std::cout << "FRAME " << i << std::endl;
        std::cout << indirectX.segment<6>(i * 6).transpose() << std::endl;
        std::cout << X.segment<6>(4 + i * 8).transpose() << std::endl;
        int numIndirectPoint = 1;
        int numDirectPoint = 0;
        std::cout << numIndirectPoint << " " << numDirectPoint << std::endl;
        scalar_t indirectRatio = (scalar_t)numIndirectPoint / (scalar_t)(numIndirectPoint + numDirectPoint);
        scalar_t directRatio = (scalar_t)1.0 - indirectRatio;
        X.segment<6>(4 + i * 8) = X.segment<6>(4 + i * 8) * directRatio + indirectX.segment<6>(i * 6) * indirectRatio;

    }
}

