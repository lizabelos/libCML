#include "cml/optimization/dso/DSOTracker.h"

typedef Sophus::SE3<CML::scalar_t> SE3;

CML::Optimization::DSOTracker::DSOTracker(Ptr<AbstractFunction, NonNullable> parent) :
AbstractFunction(parent)
{
    mFramePrivateDataInstance = getMap().getFramePrivataDataContext().createInstance();
}

CML::Optimization::DSOTracker::~DSOTracker() {

}

CML::Optimization::DSOTracker::Residual CML::Optimization::DSOTracker::optimize(int numTry, PFrame frameToTrack, PFrame reference, Camera &camera, Exposure &currentExposure, DSOTrackerContext *trackerContext) {

    if (trackerContext == nullptr) {
        trackerContext = &mTrackerContext;
    }

    auto mCD = get(reference);

    int maxIterations[] = {10, 20, 50, 50, 50};
    int maxLevel = std::min(frameToTrack->getCaptureFrame().getPyramidLevels() - 1, 4);

    Residual oldResidual, newResidual;
    oldResidual.numTermsInE.resize(maxLevel + 1, 0);
    oldResidual.numRobust.resize(maxLevel + 1, 0);
    oldResidual.numSaturated.resize(maxLevel + 1, 0);
    oldResidual.E.resize(maxLevel + 1, 0);

    newResidual.numTermsInE.resize(maxLevel + 1, 0);
    newResidual.numRobust.resize(maxLevel + 1, 0);
    newResidual.numSaturated.resize(maxLevel + 1, 0);
    newResidual.E.resize(maxLevel + 1, 0);


    List<scalar_t> levelCutoffRepeat;
    levelCutoffRepeat.resize(maxLevel + 1);

    bool haveRepeated = false;

    Camera refToNew = reference->getCamera().to(camera);

    SE3 currentRefToNew = SE3(refToNew.getRotationMatrix(), refToNew.getTranslation());
    SE3 newRefToNew;

    Exposure mNewExposure = currentExposure;

    for (int level = maxLevel; level >= 0; level--) {

      //  std::cout << "LEVEL ======================" << std::endl;

       // logger.info("Photometric tracking at level " + std::to_string(level) + " with " + std::to_string(points.size()) + " points");
/*
        if (points.size() < 100) {
            logger.info("Not enogh points. Continuing at higher level");
            continue;
        }
*/
        levelCutoffRepeat[level] = 1;

        computeResidual(frameToTrack, mCD, reference->getExposure(), currentRefToNew, currentExposure, oldResidual, level, mCutoffThreshold.f() * levelCutoffRepeat[level], trackerContext);

        while((oldResidual.numSaturated[level] / (scalar_t)oldResidual.numTermsInE[level]) > 0.6 && levelCutoffRepeat[level] < 50)
        {
            levelCutoffRepeat[level] *= 2;
            computeResidual(frameToTrack, mCD, reference->getExposure(), currentRefToNew, currentExposure, oldResidual, level, mCutoffThreshold.f() * levelCutoffRepeat[level], trackerContext);
        }

        logger.debug("Using cutoff : " + std::to_string(levelCutoffRepeat[level]) + " at level " + std::to_string(level) + " ( statured = " + std::to_string(oldResidual.numSaturated[level] / (scalar_t)oldResidual.numTermsInE[level]) + " )");

        computeHessian(frameToTrack, mCD, reference->getExposure(), currentRefToNew, currentExposure, level, trackerContext);


        scalar_t lambda = 0.01;
        scalar_t lambdaExtrapolationLimit = 0.001;

        for (int iteration = 0; iteration < maxIterations[level]; iteration++) {

            trackerContext->dampedHessian = trackerContext->hessian;
            for (int i = 0; i < 8; i++) trackerContext->dampedHessian(i, i) *= (1 + lambda);

            if (mOptimizeA.b() && mOptimizeB.b()) {
                trackerContext->increment = trackerContext->dampedHessian.ldlt().solve(-trackerContext->jacobian);
            }
            if (mOptimizeA.b() && !mOptimizeB.b()) {
                trackerContext->increment.head<7>() = trackerContext->dampedHessian.topLeftCorner<7,7>().ldlt().solve(-trackerContext->jacobian.head<7>());
                trackerContext->increment.tail<1>().setZero();
            }
            if (!mOptimizeA.b() && mOptimizeB.b()) {
                Matrix<8, 8> HlStitch = trackerContext->dampedHessian;
                Vector<8> bStitch = trackerContext->jacobian;
                HlStitch.col(6) = HlStitch.col(7);
                HlStitch.row(6) = HlStitch.row(7);
                bStitch[6] = bStitch[7];
                Vector<7> incStitch = HlStitch.topLeftCorner<7,7>().ldlt().solve(-bStitch.head<7>());
                trackerContext->increment.setZero();
                trackerContext->increment.head<6>() = incStitch.head<6>();
                trackerContext->increment[6] = 0;
                trackerContext->increment[7] = incStitch[6];
            }
            if (!mOptimizeA.b() && !mOptimizeB.b()) {
                trackerContext->increment.head<6>() = trackerContext->dampedHessian.topLeftCorner<6,6>().ldlt().solve(-trackerContext->jacobian.head<6>());
                trackerContext->increment.tail<2>().setZero();
            }

            scalar_t extrapFac = 1;
            if(lambda < lambdaExtrapolationLimit) extrapFac = sqrt(sqrt(lambdaExtrapolationLimit / lambda));
            trackerContext->increment *= extrapFac;

            trackerContext->incrementScaled = trackerContext->increment;
            trackerContext->incrementScaled.segment<3>(0) *= mScaleRotation.f();
            trackerContext->incrementScaled.segment<3>(3) *= mScaleTranslation.f();
            trackerContext->incrementScaled.segment<1>(6) *= mScaleLightA.f();
            trackerContext->incrementScaled.segment<1>(7) *= mScaleLightB.f();

            for (int i = 0; i < 8; i++) {
                if (!std::isfinite(trackerContext->incrementScaled[i])) {
                    // trackerContext->incrementScaled.setZero();
                    logger.error("Non finite DSO Tracker increment");
                    // break;
                    oldResidual.isCorrect = false;
                    return oldResidual;
                }
            }
          /*  if(!trackerContext->incrementScaled.allFinite() || trackerContext->incrementScaled.hasNaN()) {
                trackerContext->incrementScaled.setZero();
                logger.error("Non finite DSO Tracker increment");
            }
*/
            auto se3 = SE3::exp((SE3::Tangent)(trackerContext->incrementScaled.head<6>()));

            newRefToNew = se3 * currentRefToNew;
            // mNewCamera = currentCamera.compose(Camera(se3.translation(), se3.rotationMatrix()));
            mNewExposure.setParametersAndExposure(currentExposure.add(trackerContext->incrementScaled(6), trackerContext->incrementScaled(7)));

            computeResidual(frameToTrack, mCD, reference->getExposure(), newRefToNew, mNewExposure, newResidual, level, mCutoffThreshold.f() * levelCutoffRepeat[level], trackerContext);

            bool accept = (newResidual.E[level] / (scalar_t)newResidual.numTermsInE[level]) < (oldResidual.E[level] / (scalar_t)oldResidual.numTermsInE[level]);

            if (accept) {
                logger.debug("Accepting step. New residual : " + std::to_string(newResidual.rmse(level)) + "; Old residual : " + std::to_string(oldResidual.rmse(level)));
                computeHessian(frameToTrack, mCD, reference->getExposure(), newRefToNew, mNewExposure, level, trackerContext);
                oldResidual = newResidual;
                currentRefToNew = newRefToNew;
                currentExposure.setParametersAndExposure(mNewExposure);
                lambda *= 0.5;
            } else {

                lambda *= 4;
                logger.debug("Not accepting step. New residual : " + std::to_string(newResidual.rmse(level)) + "; Old residual : " + std::to_string(oldResidual.rmse(level)));
            }

            if(trackerContext->increment.norm() < 1e-3)
            {
                break;
            }

        }

        if (mLastResidual.isCorrect) {
            if(oldResidual.rmse(level) > 1.5 * mLastResidual.rmse(level)) {
                oldResidual.isCorrect = false;
                logger.debug("The solution is not good because the rmse is too high");
                return oldResidual;
            }
        }


        if(levelCutoffRepeat[level] > 1 && !haveRepeated) {
            level++;
            haveRepeated = true;
        }
            // logger.info("Repeating level");
        //} else {
        //    haveRepeated = false;
        //}

    }

    Vector2 relAff = reference->getExposure().to(currentExposure).getParameters();

    bool haveGoodLight = true;

    if (mOptimizeA.b()) {
        if (fabs(currentExposure.getParameters()(0)) > 1.2) {
            logger.debug("The solution is not good because of a");
            haveGoodLight = false;
        }
    } else {
        if (fabs(logf((float)relAff[0])) > 1.5) {
            logger.debug("The solution is not good because of a");
            haveGoodLight = false;
        }
    }

    if (mOptimizeB.b()) {
        if (fabs(currentExposure.getParameters()(1)) > 200) {
            logger.debug("The solution is not good because of b");
            haveGoodLight = false;
        }
    } else {
        if (fabs((float)relAff[1]) > 200) {
            logger.debug("The solution is not good because of b");
            haveGoodLight = false;
        }
    }

    bool haveGoodPoints = true;
    if ((scalar_t)oldResidual.numSaturated[0] / (scalar_t)oldResidual.numTermsInE[0] > mSaturatedRatioThreshold.f()) {
        haveGoodPoints = false;
        logger.info("Strange, the solution have a lot of saturated points : " + std::to_string((scalar_t)oldResidual.numSaturated[0] / (scalar_t)oldResidual.numTermsInE[0]));
    }

    camera = reference->getCamera().compose(cameraOf(currentRefToNew));

    oldResidual.isCorrect = haveGoodLight;
    oldResidual.tooManySaturated = haveGoodPoints;
    oldResidual.levelCutoffRepeat = levelCutoffRepeat;
    oldResidual.relAff = relAff;
    oldResidual.covariance = trackerContext->hessian.inverse().diagonal().head<6>();
    return oldResidual;

}

void CML::Optimization::DSOTracker::computeResidual(PFrame frameToTrack, DSOTrackerPrivate *mCD, Exposure mLastReferenceExposure, const SE3 &refToNew, Exposure exposure, Residual &result, int level, scalar_t cutoffThreshold, DSOTrackerContext *trackerContext) {

    float E = 0;
    int numTermsInE = 0;
    int numTermsInWarped = 0;
    int numSaturated=0;
    int numRobust = 0;

    int wl = frameToTrack->getWidth(level);
    int hl = frameToTrack->getHeight(level);
    // Eigen::Vector3f* dINewl = newFrame->dIp[lvl];

    Matrix33f K = frameToTrack->getK(level).cast<float>();
    Matrix33f Ki = K.inverse();
    float fxl = K(0, 0);
    float fyl = K(1, 1);
    float cxl = K(0, 2);
    float cyl = K(1, 2);

//    Camera refToNew = mCD->mLastReference->getCamera().to(camera);


    Matrix33f RKi = (refToNew.rotationMatrix().cast<float>() * Ki);
    Vector3f t = (refToNew.translation()).cast<float>();
    Vector2f affLL = mLastReferenceExposure.to(exposure).getParameters().cast<float>();

    float sumSquaredShiftT=0;
    float sumSquaredShiftRT=0;
    float sumSquaredShiftNum=0;

    float maxEnergy = 2.0f * mHuberThreshold.f() * cutoffThreshold - mHuberThreshold.f() * mHuberThreshold.f();	// energy for r=setting_coarseCutoffTH.

    DSOTrackerPrivateLevel &dataLevel = mCD->level[level];

    //int nl = mCD->mPCn[level];
    //List<float> &lpc_u = mCD->mPCu[level];
    //List<float> &lpc_v = mCD->mPCv[level];
    //List<float> &lpc_idepth = mCD->mPCidepth[level];
    //List<float> &lpc_color = mCD->mPCcolor[level];


   // std::cout << "NL : " << nl << std::endl;

    double resSum = 0;
    int resInSum = 0;

    for(int i=0;i<dataLevel.n();i++)
    {
        float id = dataLevel.PCidepth(i);
        float x = dataLevel.PCu(i);
        float y = dataLevel.PCv(i);
        float refColor = dataLevel.PCcolor(i);

        if (!std::isfinite(refColor)) {
            continue;
        }


        Vector3f pt = RKi * Vector3f(x, y, 1) + t*id;
        float u = pt[0] / pt[2];
        float v = pt[1] / pt[2];
        float Ku = fxl * u + cxl;
        float Kv = fyl * v + cyl;
        float new_idepth = id/pt[2];

        if(level == 0 && i %32 == 0)
        {
            // translation only (positive)
            Vector3f ptT = Ki * Vector3f(x, y, 1) + t * id;
            float uT = ptT[0] / ptT[2];
            float vT = ptT[1] / ptT[2];
            float KuT = fxl * uT + cxl;
            float KvT = fyl * vT + cyl;

            // translation only (negative)
            Vector3f ptT2 = Ki * Vector3f(x, y, 1) - t*id;
            float uT2 = ptT2[0] / ptT2[2];
            float vT2 = ptT2[1] / ptT2[2];
            float KuT2 = fxl * uT2 + cxl;
            float KvT2 = fyl * vT2 + cyl;

            //translation and rotation (negative)
            Vector3f pt3 = RKi * Vector3f(x, y, 1) - t*id;
            float u3 = pt3[0] / pt3[2];
            float v3 = pt3[1] / pt3[2];
            float Ku3 = fxl * u3 + cxl;
            float Kv3 = fyl * v3 + cyl;

            //translation and rotation (positive)
            //already have it.

            sumSquaredShiftT += (KuT-x)*(KuT-x) + (KvT-y)*(KvT-y);
            sumSquaredShiftT += (KuT2-x)*(KuT2-x) + (KvT2-y)*(KvT2-y);
            sumSquaredShiftRT += (Ku-x)*(Ku-x) + (Kv-y)*(Kv-y);
            sumSquaredShiftRT += (Ku3-x)*(Ku3-x) + (Kv3-y)*(Kv3-y);
            sumSquaredShiftNum+=2;
        }

        if(!(Ku > 2 && Kv > 2 && Ku < wl-3 && Kv < hl-3 && new_idepth > 0)) continue;



        Vector3f hitColor = frameToTrack->getCaptureFrame().getDerivativeImage(level).interpolate(Vector2f(Ku, Kv));

        // Vec3f hitColor = getInterpolatedElement33(dINewl, Ku, Kv, wl);
        if(!hitColor.allFinite()) {
            continue;
        }
        float residual = hitColor[0] - (float)(affLL[0] * refColor + affLL[1]);
    //    if (i < 100) std::cout << "residual with light : " << residual << " ; without light : " << hitColor[0] - refColor << std::endl;
        float hw = fabs(residual) < mHuberThreshold.f() ? 1 : mHuberThreshold.f() / fabs(residual);


        if(fabs(residual) > cutoffThreshold)
        {
            E += maxEnergy;
            numTermsInE++;
            numSaturated++;
        }
        else
        {
            E += hw *residual*residual*(2-hw);
            numTermsInE++;

            trackerContext->idepth(numTermsInWarped) = new_idepth;
            trackerContext->u(numTermsInWarped) = u;
            trackerContext->v(numTermsInWarped) = v;
            trackerContext->dx(numTermsInWarped) = hitColor[1];
            trackerContext->dy(numTermsInWarped) = hitColor[2];
            trackerContext->residual(numTermsInWarped) = residual;
            trackerContext->weight(numTermsInWarped) = hw;
            trackerContext->refcolor(numTermsInWarped) = refColor;
            numTermsInWarped++;

            resSum += residual;
            resInSum += 1;
        }

        if (fabs(residual) <= mCutoffThreshold.f()) {
            numRobust++;
        }
    }

    while(numTermsInWarped%4!=0)
    {
        trackerContext->idepth(numTermsInWarped) = 0;
        trackerContext->u(numTermsInWarped) = 0;
        trackerContext->v(numTermsInWarped) = 0;
        trackerContext->dx(numTermsInWarped) = 0;
        trackerContext->dy(numTermsInWarped) = 0;
        trackerContext->residual(numTermsInWarped) = 0;
        trackerContext->weight(numTermsInWarped) = 0;
        trackerContext->refcolor(numTermsInWarped) = 0;
        numTermsInWarped++;
    }
    trackerContext->size() = numTermsInWarped;

   // std::cout << "RES SUM : " << resSum / (double)resInSum << std::endl;

   logger.debug("Residual sum : " + std::to_string(resSum));
   logger.debug("DSO Tracker num robust : " + std::to_string(numRobust));

    result.E[level] = E;
    result.numTermsInE[level] = numTermsInE;
    result.flowVector(0) = sumSquaredShiftT/(sumSquaredShiftNum+0.1);
    result.flowVector(1) = 0;
    result.flowVector(2) = sumSquaredShiftRT/(sumSquaredShiftNum+0.1);
    result.numSaturated[level] = numSaturated;
    result.numRobust[level] = numRobust;


}

void CML::Optimization::DSOTracker::computeHessian(PFrame frameToTrack, DSOTrackerPrivate *mCD, Exposure mLastReferenceExposure, const SE3 &refToNew, Exposure exposure, int level, DSOTrackerContext *trackerContext) {
    trackerContext->accumulator().initialize();

    Matrix33 K = frameToTrack->getK(level);

    M128 fxl = K(0, 0);
    M128 fyl = K(1, 1);
    M128 b0 = mLastReferenceExposure.getParameters()(1);
    M128 a = mLastReferenceExposure.to(exposure).getParameters()(0);
    //__m128 a = _mm_set1_ps((float)(AffLight::fromToVecExposure(lastRef->ab_exposure, newFrame->ab_exposure, lastRef_aff_g2l, aff_g2l)[0]));

    M128 one = 1.0f;
    M128 minusOne = -1.0f;
    M128 zero = 0.0f;

    int n = trackerContext->size() - (trackerContext->size() % 4);

    // logger.info("Computing the hessian with " + std::to_string(n) + " points");

    assert(n%4==0);
    for(int i=0;i<n;i+=4)
    {
        M128 dx = trackerContext->dx_m128(i) * fxl;
        M128 dy = trackerContext->dy_m128(i) * fyl;
        M128 u = trackerContext->u_m128(i);
        M128 v = trackerContext->v_m128(i);
        M128 id = trackerContext->idepth_m128(i);

        trackerContext->accumulator().updateSSE_eighted(
                // Rx
                id * dx,
                // Ry
                id * dy,
                // Rz
                zero - (id * (u * dx + v * dy)),
                // tx
                zero - ((u * v * dx) + dy * (one + v * v)),
                // ty
                (u * v * dy) + (dx * (one + u * u)),
                // tz
                u * dy - v * dx,
                // a
                a * (b0 - trackerContext->refcolor_m128(i)),
                // b
                minusOne,
                // res
                trackerContext->residual_m128(i),
                // weight
                trackerContext->weight_m128(i)
                );
    }

    trackerContext->accumulator().finish();
    trackerContext->hessian = trackerContext->accumulator().H.topLeftCorner<8,8>().cast<scalar_t>() / (scalar_t)n;
    trackerContext->jacobian = trackerContext->accumulator().H.topRightCorner<8,1>().cast<scalar_t>() / (scalar_t)n;

    trackerContext->hessian.block<8,3>(0,0) *= mScaleRotation.f();
    trackerContext->hessian.block<8,3>(0,3) *= mScaleTranslation.f();
    trackerContext->hessian.block<8,1>(0,6) *= mScaleLightA.f();
    trackerContext->hessian.block<8,1>(0,7) *= mScaleLightB.f();
    trackerContext->hessian.block<3,8>(0,0) *= mScaleRotation.f();
    trackerContext->hessian.block<3,8>(3,0) *= mScaleTranslation.f();
    trackerContext->hessian.block<1,8>(6,0) *= mScaleLightA.f();
    trackerContext->hessian.block<1,8>(7,0) *= mScaleLightB.f();
    trackerContext->jacobian.segment<3>(0) *= mScaleRotation.f();
    trackerContext->jacobian.segment<3>(3) *= mScaleTranslation.f();
    trackerContext->jacobian.segment<1>(6) *= mScaleLightA.f();
    trackerContext->jacobian.segment<1>(7) *= mScaleLightB.f();



}

void CML::Optimization::DSOTracker::makeCoarseDepthL0(PFrame reference, Set<PPoint, Hasher> points) {

    auto mNewCD = create(reference);

    if (SCALEFACTOR != 2) {
        logger.fatal("Scale Factor must be 2 for DSO Tracker");
        abort();
    }

    int pyrLevelsUsed = reference->getCaptureFrame().getPyramidLevels();

    mNewCD->level.resize(pyrLevelsUsed);

    int w0 = reference->getWidth(0);
    int h0 = reference->getHeight(0);

    for (int i = 0; i < pyrLevelsUsed; i++) {

        int wl = reference->getWidth(i);
        int hl = reference->getHeight(i);

        mNewCD->level[i].resize(wl * hl);

    }

    DSOTrackerPrivateLevel &dataLevel0 = mNewCD->level[0];

    for (auto point : points) {

        //if(ph->getLastResidual(0).first != nullptr && ph->getLastResidual(0).second == DSORES_IN)
        //{
            Camera hostToTarget = point->getReferenceFrame()->getCamera().to(reference->getCamera());

            DistortedVector2d refcorner_Distorted = DistortedVector2d(point->getReferenceCorner().point(0));
            UndistortedVector2d refcorner = point->getReferenceFrame()->undistort(refcorner_Distorted, 0);
            Vector3 projectedcurp = hostToTarget.getRotationMatrix() * refcorner.homogeneous() + hostToTarget.getTranslation() * point->getReferenceInverseDepth();
            DistortedVector2d projectedcurp_Distorted = reference->distort(UndistortedVector2d(projectedcurp.hnormalized()), 0);

            scalar_t drescale = 1.0 / projectedcurp[2];

            scalar_t new_idepth = drescale * point->getReferenceInverseDepth();
            scalar_t Ku = projectedcurp_Distorted.x();
            scalar_t Kv = projectedcurp_Distorted.y();

            int u = Ku + 0.5;
            int v = Kv + 0.5;
            float weight = sqrtf(1e-3 / ( point->getUncertainty() + 1e-12));

            if (u < 0 || u >= w0) {
                continue;
            }

            if (v < 0 || v >= h0) {
                continue;
            }

            dataLevel0.idepth(u+w0*v) += new_idepth *weight;
            dataLevel0.weightSum(u+w0*v) += weight;
       // }
    }

    for(int lvl=1; lvl<pyrLevelsUsed; lvl++)
    {
        int lvlm1 = lvl-1;
        int wl = reference->getWidth(lvl);
        int hl = reference->getHeight(lvl);
        int wlm1 = reference->getWidth(lvlm1);

        DSOTrackerPrivateLevel &dataLevel = mNewCD->level[lvl];
        DSOTrackerPrivateLevel &dataLevelM1 = mNewCD->level[lvlm1];

        /*List<float>& idepth_l = mNewCD->mIdepth[lvl];
        List<float>& weightSums_l = mNewCD->mWeightSums[lvl];

        List<float>& idepth_lm = mNewCD->mIdepth[lvlm1];
        List<float>& weightSums_lm = mNewCD->mWeightSums[lvlm1];*/

        for(int y=0;y<hl;y++)
            for(int x=0;x<wl;x++)
            {
                int bidx = 2*x   + 2*y*wlm1;
                dataLevel.idepth(x + y*wl) = dataLevelM1.idepth(bidx) +
                                                dataLevelM1.idepth(bidx+1) +
                                                dataLevelM1.idepth(bidx+wlm1) +
                                                dataLevelM1.idepth(bidx+wlm1+1);

                dataLevel.weightSum(x + y*wl) = dataLevelM1.weightSum(bidx) +
                                                    dataLevelM1.weightSum(bidx+1) +
                                                    dataLevelM1.weightSum(bidx+wlm1) +
                                                    dataLevelM1.weightSum(bidx+wlm1+1);
            }
    }


    // dilate idepth by 1.
    for(int lvl=0; lvl<2; lvl++)
    {
        int numIts = 1;

        DSOTrackerPrivateLevel &dataLevel = mNewCD->level[lvl];

        for(int it=0;it<numIts;it++)
        {
            int wl = reference->getWidth(lvl);
            int hl = reference->getHeight(lvl);
            int wh = wl * hl - wl;

            //List<float>& weightSumsl = mNewCD->mWeightSums[lvl];
            //List<float>& weightSumsl_bak = mNewCD->mWeightSumsBak[lvl];
            dataLevel.backupWeightSum();
            // weightSumsl_bak = weightSumsl;
            // memcpy(weightSumsl_bak, weightSumsl, wl*hl*sizeof(float));
            //List<float>& idepthl = mNewCD->mIdepth[lvl];	// dotnt need to make a temp copy of depth, since I only
            // read values with weightSumsl>0, and write ones with weightSumsl<=0.
            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for(int i=wl;i<wh;i++)
            {
                if(dataLevel.weightSumBak(i) <= 0)
                {
                    float sum=0, num=0, numn=0;
                    if(i+1+wl >= 0 && i+1+wl < (int)dataLevel.size() && dataLevel.weightSumBak(i+1+wl) > 0) { sum += dataLevel.idepth(i+1+wl); num+=dataLevel.weightSumBak(i+1+wl); numn++;}
                    if(i-1-wl >= 0 && i-1-wl < (int)dataLevel.size() && dataLevel.weightSumBak(i-1-wl) > 0) { sum += dataLevel.idepth(i-1-wl); num+=dataLevel.weightSumBak(i-1-wl); numn++;}
                    if(i+wl-1 >= 0 && i+wl-1 < (int)dataLevel.size() && dataLevel.weightSumBak(i+wl-1) > 0) { sum += dataLevel.idepth(i+wl-1); num+=dataLevel.weightSumBak(i+wl-1); numn++;}
                    if(i-wl+1 >= 0 && i-wl+1 < (int)dataLevel.size() && dataLevel.weightSumBak(i-wl+1) > 0) { sum += dataLevel.idepth(i-wl+1); num+=dataLevel.weightSumBak(i-wl+1); numn++;}
                    if(numn > 0) {
                        dataLevel.idepth(i) = sum/numn;
                        dataLevel.weightSum(i) = num/numn;
                    }
                }
            }
        }
    }


    // dilate idepth by 1 (2 on lower levels).
    for(int lvl=2; lvl<pyrLevelsUsed; lvl++)
    {
        int wl = reference->getWidth(lvl);
        int hl = reference->getHeight(lvl);
        int wh = wl * hl - wl;

        DSOTrackerPrivateLevel &dataLevel = mNewCD->level[lvl];

        //List<float>& weightSumsl = mNewCD->mWeightSums[lvl];
        //List<float>& weightSumsl_bak = mNewCD->mWeightSumsBak[lvl];

        //memcpy(weightSumsl_bak, weightSumsl, w[lvl]*h[lvl]*sizeof(float));
        dataLevel.backupWeightSum();
        //weightSumsl_bak = weightSumsl;
        //List<float>& idepthl = mNewCD->mIdepth[lvl];	// dotnt need to make a temp copy of depth, since I only
        // read values with weightSumsl>0, and write ones with weightSumsl<=0.
        #if CML_USE_OPENMP
        #pragma omp parallel for schedule(static)
        #endif
        for(int i=wl;i<wh;i++)
        {
            if(dataLevel.weightSumBak(i) <= 0)
            {
                float sum=0, num=0, numn=0;
                if(i+1 >= 0 && i+1 < (int)dataLevel.size() && dataLevel.weightSumBak(i+1) > 0) { sum += dataLevel.idepth(i+1); num+=dataLevel.weightSumBak(i+1); numn++;}
                if(i-1 >= 0 && i-1 < (int)dataLevel.size() && dataLevel.weightSumBak(i-1) > 0) { sum += dataLevel.idepth(i-1); num+=dataLevel.weightSumBak(i-1); numn++;}
                if(i+wl >= 0 && i+wl < (int)dataLevel.size() && dataLevel.weightSumBak(i+wl) > 0) { sum += dataLevel.idepth(i+wl); num+=dataLevel.weightSumBak(i+wl); numn++;}
                if(i-wl >= 0 && i-wl < (int)dataLevel.size() && dataLevel.weightSumBak(i-wl) > 0) { sum += dataLevel.idepth(i-wl); num+=dataLevel.weightSumBak(i-wl); numn++;}
                if(numn>0) {
                    dataLevel.idepth(i) = sum/numn;
                    dataLevel.weightSum(i) = num/numn;
                }
            }
        }
    }


    // normalize idepths and weights.
    for(int lvl=0; lvl<pyrLevelsUsed; lvl++)
    {

        DSOTrackerPrivateLevel &dataLevel = mNewCD->level[lvl];

        //List<float>& weightSumsl = mNewCD->mWeightSums[lvl];
        //List<float>& idepthl = mNewCD->mIdepth[lvl];
        //Eigen::Vector3f* dIRefl = lastRef->dIp[lvl];

        int wl = reference->getWidth(lvl);
        int hl = reference->getHeight(lvl);

        int lpc_n=0;
        //List<float>& lpc_u = mNewCD->mPCu[lvl];
        //List<float>& lpc_v = mNewCD->mPCv[lvl];
        //List<float>& lpc_idepth = mNewCD->mPCidepth[lvl];
        //List<float>& lpc_color = mNewCD->mPCcolor[lvl];


        for(int y=2;y<hl-2;y++)
            for(int x=2;x<wl-2;x++)
            {
                int i = x+y*wl;

                if(dataLevel.weightSum(i) > 0)
                {
                    dataLevel.idepth(i) /= dataLevel.weightSum(i);
                    float idepth = dataLevel.idepth(i); // For the cache

                    dataLevel.PCu(lpc_n) = x;
                    dataLevel.PCv(lpc_n) = y;
                    dataLevel.PCidepth(lpc_n) = idepth;
                    // lpc_color[lpc_n] = dIRefl[i][0];
                    dataLevel.PCcolor(lpc_n) = reference->getCaptureFrame().getGrayImage(lvl).get(x, y); // todo : for the cache, maybe copy the image side by side to the data


                    if(!std::isfinite(dataLevel.PCcolor(lpc_n)) || !(idepth>0))
                    {
                        dataLevel.idepth(i) = -1;
                        continue;	// just skip if something is wrong.
                    }
                    lpc_n++;
                }
                else
                    dataLevel.idepth(i) = -1;

                dataLevel.weightSum(i) = 1;
            }

        dataLevel.n() = lpc_n;
    }

    mLastComputed = reference;


}

void CML::Optimization::DSOTracker::viewOnCapture(DrawBoard &drawBoard, PFrame frame) {
    /*
    if (mCD->mPCn.size() == 0) {
        return;
    }

   int lvl = 0;
   int wl = mCD->mLastReference->getWidth(lvl);
   int hl = mCD->mLastReference->getHeight(lvl);

   List<float>& idepth_l = mCD->mIdepth[lvl];
   List<float>& weightSums_l = mCD->mWeightSums[lvl];

   float idepthMax = 0.0001;
   for(int y=0;y<hl;y++) {
       for (int x = 0; x < wl; x++) {
           idepthMax = std::max(idepth_l[x + y * wl], idepthMax);
       }
   }

    for(int y=0;y<hl;y++) {
        for (int x = 0; x < wl; x++) {
            if (idepth_l[x + y * wl] > 0) {
                drawBoard.pointSize(1);
                drawBoard.color(1 - (idepth_l[x + y * wl] / idepthMax), 0, idepth_l[x + y * wl] / idepthMax);
                drawBoard.point(Vector2f(x, y));
            }
        }
    }
*/
}

void CML::Optimization::DSOTracker::viewOnReconstruction(DrawBoard &drawBoard) {
    drawBoard.disableDepthTest();

    List<Camera> lastTriedCamera, lastOptimizedCamera;
    {
        LockGuard lg(mLastCameraMutex);
        lastTriedCamera = mLastTriedCamera;
        lastOptimizedCamera = mLastOptimizedCamera;
    }

    for (auto camera : lastTriedCamera) {
        drawBoard.color(1,0,0);
        drawBoard.paintCamera(camera);
    }

    for (auto camera : lastOptimizedCamera) {
        drawBoard.color(0,0,1);
        drawBoard.paintCamera(camera);
    }

    drawBoard.enableDepthTest();
}

