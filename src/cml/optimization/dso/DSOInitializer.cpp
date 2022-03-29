#include "cml/optimization/dso/DSOInitializer.h"
#include "cml/utils/KDTree.h"
#include "cml/maths/Utils.h"

typedef Sophus::SE3<CML::scalar_t> SE3;

bool CML::Optimization::DSOInitializer::setFirst(PFrame reference) {
    LockGuard lg(mPointsMutex);

    mReference = reference;
    mNumPyramidLevel = std::min(5, reference->getCaptureFrame().getPyramidLevels());

    mPixelSelector = new Features::PixelSelector(this, reference->getWidth(0), reference->getHeight(0));

    Array2D<float> statusMap;
    Array2D<bool> statusMapB;

    mPoints = List<List<DSOInitializerPoint>>();
    mPoints.resize(mNumPyramidLevel);

    int w0 = reference->getWidth(0), h0 = reference->getHeight(0);


    float densitiesFactor = mDensityFactor.f();
    float densities[] = {
            densitiesFactor * 0.03f,
            densitiesFactor * 0.05f,
            densitiesFactor * 0.15f,
            densitiesFactor * 0.5f,
            densitiesFactor * 1.0f};

    for(int lvl = 0; lvl < mNumPyramidLevel; lvl++)
    {
        int wl = reference->getWidth(lvl), hl = reference->getHeight(lvl);
        mPixelSelector->setPotential(3);
        if(lvl == 0) {
            int n = mPixelSelector->makeMaps(reference->getCaptureFrame(), statusMap, densities[lvl] * wl * hl, 1, 2);
        }
        else {
            mPixelSelector->makePixelStatus(reference->getCaptureFrame(), lvl, statusMapB, densities[lvl] * w0 * h0, mSparsityFactor);
        }

        // set idepth map to initially 1 everywhere.
        int nl = 0;
        const int patternPadding = 2; // todo
        for(int y=patternPadding+1;y<hl-patternPadding-2;y++) {
            for (int x = patternPadding + 1; x < wl - patternPadding - 2; x++) {
                //if(x==2) printf("y=%d!\n",y);
                if ((lvl != 0 && statusMapB(x, y)) || (lvl == 0 && statusMap(x, y) != 0)
                && reference->getCaptureFrame().getDerivativeImage(0).get(x, y).allFinite()) {
                    DSOInitializerPoint p;
                    //assert(patternNum==9);

                    p.p.x() = x+0.1;
                    p.p.y() = y+0.1;
                    p.idepth = 1;
                    p.iR = 1;
                    p.isGood = true;
                    p.energy.setZero();
                    p.lastHessian = 0;
                    p.lastHessian_new = 0;
                    p.my_type = (lvl != 0) ? 1 : statusMap(x, y);

                    for(size_t idx = 0; idx < mPattern.size(); idx++) {
                        Vector2f pos = p.p + mPattern[idx].cast<float>();
                        p.pPattern[idx] = pos.homogeneous();
                        p.color[idx] = reference->getCaptureFrame().getGrayImage(lvl).interpolate(pos);
                    }

                    //Eigen::Vector3f *cpt = firstFrame->dIp[lvl] + x + y * w[lvl];
                    float sumGrad2 = 0;
                    for (size_t idx = 0; idx < mPattern.size(); idx++) {
                        //int dx = patternP[idx][0];
                        //int dy = patternP[idx][1];
                        //float absgrad = cpt[dx + dy * w[lvl]].tail<2>().squaredNorm();
                        float absgrad = reference->getCaptureFrame().getDerivativeImage(lvl).get(x + mPattern[idx].x(), y + mPattern[idx].y()).tail<2>().squaredNorm();
                        sumGrad2 += absgrad;
                    }

//				float gth = setting_outlierTH * (sqrtf(sumGrad2)+setting_outlierTHSumComponent);
//				pl[nl].outlierTH = patternNum*gth*gth;
//

                    p.outlierTH = mPattern.size() * mSettingOutlierTH.f();

                    mPoints[lvl].emplace_back(p);
                    nl++;

                }
            }
        }

        logger.info(std::to_string(mPoints[lvl].size()) + " points at level " + std::to_string(lvl));
        if (mPoints[lvl].size() < 10) {
            return false;
        }
    }

    makeNN();

    mFrameID = 0;
    mSnapped = false;
    mSnappedAt = 0;

    mReference = reference;

    return true;

}

int CML::Optimization::DSOInitializer::tryInitialize(PFrame frameToTrack, PFrame reference) {
    mNNWeight = 0;
    Array2D<float> depthMap(reference->getWidth(0), reference->getHeight(0), 1.0f);
    return tryInitialize(frameToTrack, reference, depthMap);
}

int CML::Optimization::DSOInitializer::tryInitialize(PFrame frameToTrack, PFrame reference, const Array2D<float> &inverseDepthMap) {
    assertThrow(reference != frameToTrack, "The reference frame cannot be equal to the frame to track");

   // auto referenceData = mContext.get(reference);
   // auto frameToTrackData = mContext.get(frameToTrack);

    //referenceData->ab_exposure = reference->getCaptureFrame().getExposureTime();
    //frameToTrackData->ab_exposure = frameToTrack->getCaptureFrame().getExposureTime();


    if (mReference != reference || !mIsInit) {
        logger.info("Initializing initializer");
        mIsInit = setFirst(reference);
        mCurrentCamera = frameToTrack->getCamera();
        mCurrentExposure.setParametersAndExposure(frameToTrack->getExposure());
        mFrames.clear();
    } else {
        frameToTrack->setExposureParameters(mCurrentExposure);
        mCurrentExposure.setParametersAndExposure(frameToTrack->getExposure());
    }

    if (!mIsInit) {
        return 0;
    }

    mFrames.emplace_back(frameToTrack);

    mJbBuffer.resize(frameToTrack->getWidth(0) * frameToTrack->getHeight(0));
    mJbBuffer_new.resize(frameToTrack->getWidth(0) * frameToTrack->getHeight(0));

    bool fixAffine = true;

    Eigen::DiagonalMatrix<float, 8> wM;
    wM.diagonal()[0] = wM.diagonal()[1] = wM.diagonal()[2] = mScaleRotation.f();
    wM.diagonal()[3] = wM.diagonal()[4] = wM.diagonal()[5] = mScaleTranslation.f();
    wM.diagonal()[6] = mScaleLightA.f();
    wM.diagonal()[7] = mScaleLightB.f();

    mReference = reference;
    mFrameToTrack = frameToTrack;

    assertThrow(mPoints[0].size() > 0, "No points...");
    logger.info("Initializing using " + std::to_string(mPoints[0].size()) + " points");

    makeNN();


    List<int> maxIterations;
    maxIterations.resize(std::max(mNumPyramidLevel, 4), 50);
    maxIterations[0] = 5;
    maxIterations[1] = 5;
    maxIterations[2] = 10;
    maxIterations[3] = 30;


    mAlphaK = 2.5*2.5;
    mAlphaW = 150*150;
    mRegWeight = mRegulalizationWeight.f();
    mCouplingWeight = 1;

    if(!mSnapped)
    {
        mCurrentCamera = mCurrentCamera.withTranslation(mReference->getCamera().getTranslation());
        for(int lvl = 0; lvl < mNumPyramidLevel; lvl++)
        {
            for(size_t i = 0; i < mPoints[lvl].size(); i++)
            {
                auto &pointData = mPoints[lvl][i];

                scalar_t x = pointData.x() * (scalar_t)inverseDepthMap.getWidth() / (scalar_t)mReference->getWidth(lvl);
                scalar_t y = pointData.y() * (scalar_t)inverseDepthMap.getHeight() / (scalar_t)mReference->getHeight(lvl);
                scalar_t depth = inverseDepthMap.interpolate(Vector2f(x, y));

                pointData.initialiR = depth;
                pointData.iR = depth;
                pointData.idepth_new = 1;
                pointData.lastHessian = 0;
            }
        }
    }

    Vector3f latestRes = Vector3f::Zero();
    for(int lvl = mNumPyramidLevel - 1; lvl>=0; lvl--)
    {

        if(lvl < mNumPyramidLevel - 1) {
            propagateDown(lvl + 1);
        }

        Matrixf<8,8> H,Hsc; Vector8f b,bsc;
        resetPoints(lvl);
        Vector3f resOld = calcResAndGS(lvl, H, b, Hsc, bsc, mCurrentCamera, mCurrentExposure);
        if (resOld[2] == 0) {
            return -1;
        }
        applyStep(lvl);

        float lambda = 0.1;
        float eps = 1e-4;
        int fails=0;

        int iteration=0;
        while(true)
        {
            Matrixf<8,8> Hl = H;
            for(int i=0;i<8;i++) Hl(i,i) *= (1+lambda);
            Hl -= Hsc*(1/(1+lambda));
            Vector8f bl = b - bsc*(1/(1+lambda));

            Hl = wM * Hl * wM * (0.01f/(float)(frameToTrack->getWidth(lvl)*frameToTrack->getHeight(lvl)));
            bl = wM * bl * (0.01f/(float)(frameToTrack->getWidth(lvl)*frameToTrack->getHeight(lvl)));

            Vector8f inc;
            if(fixAffine)
            {
                inc.head<6>() = - (wM.toDenseMatrix().topLeftCorner<6,6>() * (Hl.topLeftCorner<6,6>().ldlt().solve(bl.head<6>())));
                inc.tail<2>().setZero();
            }
            else {
                inc = -(wM * (Hl.ldlt().solve(bl)));    //=-H^-1 * b.
            }

            auto se3 = SE3::exp((SE3::Tangent)(inc.head<6>().cast<scalar_t>()));

            Camera newCamera = mCurrentCamera.compose(Camera(se3.translation(), Quaternion(se3.unit_quaternion().w(), se3.unit_quaternion().x(), se3.unit_quaternion().y(), se3.unit_quaternion().z())));
            Exposure newExposure = mCurrentExposure.add(inc[6], inc[7]);

            doStep(lvl, lambda, inc);


            Matrixf<8,8> H_new, Hsc_new; Vector8f b_new, bsc_new;
            Vector3f resNew = calcResAndGS(lvl, H_new, b_new, Hsc_new, bsc_new, newCamera, newExposure);
            if (resNew[2] == 0) {
                return -1;
            }

            Vector3f regEnergy = calcEC(lvl);


            float eTotalNew = (resNew[0]+resNew[1]+regEnergy[1]);
            float eTotalOld = (resOld[0]+resOld[1]+regEnergy[0]);


            bool accept = eTotalOld > eTotalNew;

            if(accept)
            {

                logger.info(std::to_string(mStepNum) + "; Accepting the step : " + std::to_string(resNew[0] / resNew[2]) + " < " + std::to_string(resOld[0] / resOld[2]) + ". Level = " + std::to_string(lvl));

                if(resNew[1] == mAlphaK * mPoints[lvl].size()) {
                    mSnapped = true;
                }
                H = H_new;
                b = b_new;
                Hsc = Hsc_new;
                bsc = bsc_new;
                resOld = resNew;
                mCurrentCamera = newCamera;
                mCurrentExposure.setParametersAndExposure(newExposure);

                frameToTrack->setCamera(mCurrentCamera);
                frameToTrack->setExposureParameters(mCurrentExposure);

                applyStep(lvl);
                optReg(lvl);
                lambda *= 0.5;
                fails=0;
                if(lambda < 0.0001) lambda = 0.0001;
                mStepNum++;

            }
            else
            {
                logger.info(std::to_string(mStepNum) + "; Not accepting the step : " + std::to_string(resNew[0] / resNew[2]) + " > " + std::to_string(resOld[0] / resOld[2]) + ". Level = " + std::to_string(lvl));


                fails++;
                lambda *= 4;
                if(lambda > 10000) lambda = 10000;
                mStepNum++;
            }

            bool quitOpt = false;

            if(!(inc.norm() > eps) || iteration >= maxIterations[lvl] || fails >= 2)
            {
                Matrixf<8,8> H,Hsc; Vector8f b,bsc;

                quitOpt = true;
            }


            if(quitOpt) break;
            iteration++;
        }
        latestRes = resOld;

    }

    for(int i = 0; i < mNumPyramidLevel - 1; i++) {
        propagateUp(i);
    }

    mFrameID++;
    if(!mSnapped) {
        mSnappedAt = 0;
    }

    if(mSnapped && mSnappedAt == 0) {
        mSnappedAt = mFrameID;
    }

    mSuccess =  mSnapped && mFrameID > mSnappedAt + 5;

    if (mSuccess) {

        onInitializationSuccess();

    } else {

        frameToTrack->setCamera(mCurrentCamera);
        frameToTrack->setExposureParameters(mCurrentExposure);


    }

    if (mSuccess) {
        return 1;
    } else {
        return 0;
    }
}

void CML::Optimization::DSOInitializer::onInitializationSuccess() {

    PFrame frameToTrack = mFrameToTrack;
    PFrame reference = mReference;

  //  auto frameToTrackData = mContext.get(frameToTrack);
  //  auto referenceData = mContext.get(reference);

    float minir = 999;

    float sumID=1e-5, numID=1e-5;
    List<float> allIR;
    for(size_t i = 0;i < mPoints[0].size(); i++)
    {
        auto &pointData = mPoints[0][i];
        if (!pointData.isGood) {
            continue;
        }
        sumID += pointData.iR;
        numID++;
        allIR.emplace_back(pointData.iR);
        minir = std::min(minir, pointData.iR);
    }
    float rescaleFactor = 0.5f / median(allIR);


   /* for (auto frame : mFrames) {

        if (frame == frameToTrack || frame == reference) {
            continue;
        }

        frame->addDeform(reference, frameToTrack);
    }
*/
    mCurrentCamera = mCurrentCamera.withTranslation(mCurrentCamera.getTranslation() / rescaleFactor);
    for (auto frame : mFrames) {

        if (frame == frameToTrack || frame == reference) {
            continue;
        }

        frame->setCamera(frame->getCamera().withTranslation(frame->getCamera().getTranslation() / rescaleFactor));

    }

    frameToTrack->setCamera(mCurrentCamera);
    frameToTrack->setExposureParameters(mCurrentExposure);

   // referenceData->setEvalPT_scaled(Sophus::SE3<CML::scalar_t>(reference->getCamera().getRotationMatrix(), reference->getCamera().getTranslation()), reference->getExposure(), mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());
   // frameToTrackData->setEvalPT_scaled(Sophus::SE3<CML::scalar_t>(frameToTrack->getCamera().getRotationMatrix(), frameToTrack->getCamera().getTranslation()), frameToTrack->getExposure(), mScaleTranslation.f(), mScaleRotation.f(), mScaleLightA.f(), mScaleLightB.f());

    //float keepPercentage = (float)mSettingsDesiredPointDensity.i() / (float)mPoints[0].size();

    List<Corner> corners;
    for (size_t i = 0;i < mPoints[0].size(); i++) {
        auto &pointInitializerData = mPoints[0][i];
        corners.emplace_back(Corner(DistortedVector2d(pointInitializerData.x() + 0.5, pointInitializerData.y() + 0.5)));
    }
    int groupId = reference->addFeaturePoints(corners);

    size_t desiredPointDensity = std::min((int)mPoints[0].size(), mSettingsDesiredPointDensity.i());
    for (size_t j = 0; j < desiredPointDensity; j++) {

        size_t i = j * mPoints[0].size() / desiredPointDensity;
        //float randomValue = (float)rand() / (float)RAND_MAX;
        //if(randomValue > keepPercentage) continue;

        auto &pointInitializerData = mPoints[0][i];

        if (!pointInitializerData.isGood) {
            continue;
        }

        auto point = getMap().createMapPoint(reference, FeatureIndex(groupId, i), DIRECT);
        reference->addDirectApparitions(point);

       // auto pointGlobalData = mContext.get(point);

//        pointGlobalData->iDepthMin = pointGlobalData->iDepthMax = 1;
        point->setReferenceInverseDepth(pointInitializerData.iR * rescaleFactor);
        mFrameToTrack->addDirectApparitions(point);

    }

    // Free memory
   /* mReference = nullptr;
    mJbBuffer = List<Vectorf<10>>();
    mJbBuffer_new = List<Vectorf<10>>();

    LockGuard lg(mPointsMutex);
    mPoints = List<List<DSOInitializerPoint>>();
*/
}

CML::Vector3f CML::Optimization::DSOInitializer::calcResAndGS(int lvl, Matrixf<8, 8> &H_out, Vector8f &b_out, Matrixf<8, 8> &H_out_sc, Vector8f &b_out_sc, const Camera &camera, const Exposure &exposure) {

    int numOutliers = 0, numInliers = 0;

    int wl = mFrameToTrack->getWidth(lvl), hl = mFrameToTrack->getHeight(lvl);
    Matrix33 K = mFrameToTrack->getK(lvl);
    Matrix33 Ki = K.inverse();
    float fxl = K(0, 0);
    float fyl = K(1, 1);
    float cxl = K(0, 2);
    float cyl = K(1, 2);

    int npts = mPoints[lvl].size();

    //Eigen::Vector3f* colorRef = firstFrame->dIp[lvl];
    //Eigen::Vector3f* colorNew = newFrame->dIp[lvl];

    Camera refToNew = mReference->getCamera().to(camera);
    // ExposureTransition refToNewExposure = mReference->getExposure().to(exposure);

    Matrix33f RKi = (refToNew.getRotationMatrix() * Ki).cast<float>();
    Vector3f t = refToNew.getTranslation().cast<float>();
    //Eigen::Vector2f r2new_aff = refToNewExposure.getParameters().cast<float>();
    Vector2f r2new_aff = Vector2f(
             exposure.getExposureFromCamera() /  mReference->getExposure().getExposureFromCamera(),
            0
            );

    dso::Accumulator11 E;
    mAcc9.initialize();
    E.initialize();

    for (int i = 0; i < npts; i++)
    {
        for(size_t idx = 0; idx < mPattern.size(); idx++)
        {
            auto point = &mPoints[lvl][i];

            // UndistortedVector2d undistorted = mReference->undistort(DistortedVector2d(Vector2(point->u, point->v) + Vector2(mPattern[idx].x(), mPattern[idx].y())),lvl);
            // Vector3 pt = WorldPoint::fromInverseDepth(point->idepth_new, undistorted, mReference->getCamera()).relative(camera);
            point->tempPt[idx].setZero();
            point->tempPt[idx].noalias() += RKi * point->pPattern[idx];
            point->tempPt[idx].noalias() += t * point->idepth_new;

            point->tempU[idx] = point->tempPt[idx][0] / point->tempPt[idx][2];
            point->tempV[idx] = point->tempPt[idx][1] / point->tempPt[idx][2];
            point->tempKu[idx] = fxl * point->tempU[idx] + cxl;
            point->tempKv[idx] = fyl * point->tempV[idx] + cyl;
            point->tempNewIdepth[idx] = point->idepth_new/point->tempPt[idx][2];

            if(point->tempKu[idx] > 1 && point->tempKv[idx] > 1 &&  point->tempKu[idx] < wl-2 &&  point->tempKv[idx] < hl-2 && point->tempNewIdepth[idx] > 0)
            {
                // isGood = false;
                point->hitColors[idx] = mFrameToTrack->getCaptureFrame().getDerivativeImage(lvl).interpolate(Vector2f( point->tempKu[idx],  point->tempKv[idx]));

            } else {
                point->isGood = false;
            }


        }
    }

    for (int pti = 0; pti < npts; pti++)
    {
        auto point = &mPoints[lvl][pti];

        point->maxstep = 1e10;
        if(!point->isGood)
        {
            E.updateSingle((float)(point->energy[0]));
            point->energy_new = point->energy;
            point->isGood_new = false;
            continue;
        }

#define MAX_RES_PER_POINT 8

        Vectorf<MAX_RES_PER_POINT> dp0;
        Vectorf<MAX_RES_PER_POINT> dp1;
        Vectorf<MAX_RES_PER_POINT> dp2;
        Vectorf<MAX_RES_PER_POINT> dp3;
        Vectorf<MAX_RES_PER_POINT> dp4;
        Vectorf<MAX_RES_PER_POINT> dp5;
        Vectorf<MAX_RES_PER_POINT> dp6;
        Vectorf<MAX_RES_PER_POINT> dp7;
        Vectorf<MAX_RES_PER_POINT> dd;
        Vectorf<MAX_RES_PER_POINT> r;
        mJbBuffer_new[pti].setZero();

        // sum over all residuals.
        bool isGood = true;
        float energy=0;
        for(size_t idx = 0; idx < mPattern.size(); idx++)
        {
            // UndistortedVector2d undistorted = mReference->undistort(DistortedVector2d(Vector2(point->u, point->v) + Vector2(mPattern[idx].x(), mPattern[idx].y())),lvl);
            // Vector3 pt = WorldPoint::fromInverseDepth(point->idepth_new, undistorted, mReference->getCamera()).relative(camera);
            /*Vector3f pt = RKi * point->pPattern[idx] + t*point->idepth_new;

            float u = pt[0] / pt[2];
            float v = pt[1] / pt[2];
            float Ku = fxl * u + cxl;
            float Kv = fyl * v + cyl;
            float new_idepth = point->idepth_new/pt[2];

            if(!(Ku > 1 && Kv > 1 && Ku < wl-2 && Kv < hl-2 && new_idepth > 0))
            {
                isGood = false;
                break;
            }*/


            Vector3f &hitColor = point->hitColors[idx];

            //float rlR = colorRef[point->u+dx + (point->v+dy) * wl][0];
            // float rlR = mReference->getCaptureFrame().getGrayImage(lvl).interpolate(Vector2(point->u + dx, point->v + dy));
            float rlR = point->color[idx];

            if(!std::isfinite(rlR) || !std::isfinite((float)hitColor[0]) || !std::isfinite((float)hitColor[1]) || !std::isfinite((float)hitColor[2]))
            {
                isGood = false;
                break;
            }


            float residual = hitColor[0] - r2new_aff[0] * rlR - r2new_aff[1];
            // The transition goes from reference to target
           // float residual = hitColor[0] - refToNewExposure(rlR);
            float hw = fabs(residual) < mHuberThreshold.f() ? 1 : mHuberThreshold.f() / fabs(residual);
            energy += hw *residual*residual*(2-hw);

            float dxdd = (t[0]-t[2]*point->tempU[idx])/point->tempPt[idx][2];
            float dydd = (t[1]-t[2]*point->tempV[idx])/point->tempPt[idx][2];

            if(hw < 1) hw = sqrtf(hw);
            float dxInterp = hw*hitColor[1]*fxl;
            float dyInterp = hw*hitColor[2]*fyl;
            dp0[idx] = point->tempNewIdepth[idx]*dxInterp;
            dp1[idx] = point->tempNewIdepth[idx]*dyInterp;
            dp2[idx] = -point->tempNewIdepth[idx]*(point->tempU[idx]*dxInterp + point->tempV[idx]*dyInterp);
            dp3[idx] = -point->tempU[idx]*point->tempV[idx]*dxInterp - (1+point->tempV[idx]*point->tempV[idx])*dyInterp;
            dp4[idx] = (1+point->tempU[idx]*point->tempU[idx])*dxInterp + point->tempU[idx]*point->tempV[idx]*dyInterp;
            dp5[idx] = -point->tempV[idx]*dxInterp + point->tempU[idx]*dyInterp;
            dp6[idx] = - hw*r2new_aff[0] * rlR;
            dp7[idx] = - hw*1;
            dd[idx] = dxInterp * dxdd  + dyInterp * dydd;
            r[idx] = hw*residual;

            float maxstep = 1.0f / Vector2f(dxdd * fxl, dydd * fyl).norm();
            if(maxstep < point->maxstep) point->maxstep = maxstep;

            // immediately compute dp*dd' and dd*dd' in JbBuffer1.
            mJbBuffer_new[pti][0] += dp0[idx]*dd[idx];
            mJbBuffer_new[pti][1] += dp1[idx]*dd[idx];
            mJbBuffer_new[pti][2] += dp2[idx]*dd[idx];
            mJbBuffer_new[pti][3] += dp3[idx]*dd[idx];
            mJbBuffer_new[pti][4] += dp4[idx]*dd[idx];
            mJbBuffer_new[pti][5] += dp5[idx]*dd[idx];
            mJbBuffer_new[pti][6] += dp6[idx]*dd[idx];
            mJbBuffer_new[pti][7] += dp7[idx]*dd[idx];
            mJbBuffer_new[pti][8] += r[idx]*dd[idx];
            mJbBuffer_new[pti][9] += dd[idx]*dd[idx]; // idepth hessian

        }

        if(!isGood || energy > point->outlierTH * 20)
        {
            numOutliers++;
            E.updateSingle((float)(point->energy[0]));
            point->isGood_new = false;
            point->energy_new = point->energy;
            continue;
        }
        numInliers++;


        // add into energy.
        E.updateSingle(energy);
        point->isGood_new = true;
        point->energy_new[0] = energy;

        // update Hessian matrix.
        for(size_t i = 0; i + 3 < mPattern.size(); i += 4) {
            mAcc9.updateSSE(
                    _mm_load_ps(((float *) (&dp0)) + i),
                    _mm_load_ps(((float *) (&dp1)) + i),
                    _mm_load_ps(((float *) (&dp2)) + i),
                    _mm_load_ps(((float *) (&dp3)) + i),
                    _mm_load_ps(((float *) (&dp4)) + i),
                    _mm_load_ps(((float *) (&dp5)) + i),
                    _mm_load_ps(((float *) (&dp6)) + i),
                    _mm_load_ps(((float *) (&dp7)) + i),
                    _mm_load_ps(((float *) (&r)) + i));
        }


        for(size_t i = ((mPattern.size()>>2)<<2); i < mPattern.size(); i++) {
            mAcc9.updateSingle(
                    (float) dp0[i], (float) dp1[i], (float) dp2[i], (float) dp3[i],
                    (float) dp4[i], (float) dp5[i], (float) dp6[i], (float) dp7[i],
                    (float) r[i]);
        }


    }

    E.finish();
    mAcc9.finish();






    // calculate alpha energy, and decide if we cap it.
    dso::Accumulator11 EAlpha;
    EAlpha.initialize();
    for(int i=0;i<npts;i++)
    {
        auto point = &mPoints[lvl][i];
        if(!point->isGood_new)
        {
            E.updateSingle((float)(point->energy[1]));
        }
        else
        {
            point->energy_new[1] = (point->idepth_new-1)*(point->idepth_new-1);
            E.updateSingle((float)(point->energy_new[1]));
        }
    }
    EAlpha.finish();
    float alphaEnergy = mAlphaW*(EAlpha.A + refToNew.getTranslation().squaredNorm() * npts);

    mStatisticAlphaEnergy->addValue(alphaEnergy / (scalar_t)npts);

    //printf("AE = %f * %f + %f\n", alphaW, EAlpha.A, refToNew.translation().squaredNorm() * npts);


    // compute alpha opt.
    float alphaOpt;
    if(alphaEnergy > mAlphaK*npts)
    {
        alphaOpt = 0;
        alphaEnergy = mAlphaK*npts;
    }
    else
    {
        alphaOpt = mAlphaW;
    }


    mAcc9SC.initialize();
    for(int i=0;i<npts;i++)
    {
        auto point = &mPoints[lvl][i];
        if(!point->isGood_new)
            continue;

        point->lastHessian_new = mJbBuffer_new[i][9];

        mJbBuffer_new[i][8] += alphaOpt*(point->idepth_new - 1);
        mJbBuffer_new[i][9] += alphaOpt;

        if(alphaOpt==0)
        {
            mJbBuffer_new[i][8] += mCouplingWeight*(point->idepth_new - point->iR);
            mJbBuffer_new[i][9] += mCouplingWeight;
        }

        mJbBuffer_new[i][9] = 1/(1+mJbBuffer_new[i][9]);
        mAcc9SC.updateSingleWeighted(
                (float)mJbBuffer_new[i][0],(float)mJbBuffer_new[i][1],(float)mJbBuffer_new[i][2],(float)mJbBuffer_new[i][3],
                (float)mJbBuffer_new[i][4],(float)mJbBuffer_new[i][5],(float)mJbBuffer_new[i][6],(float)mJbBuffer_new[i][7],
                (float)mJbBuffer_new[i][8],(float)mJbBuffer_new[i][9]);
    }
    mAcc9SC.finish();


    //printf("nelements in H: %d, in E: %d, in Hsc: %d / 9!\n", (int)acc9.num, (int)E.num, (int)acc9SC.num*9);
    H_out = mAcc9.H.topLeftCorner<8,8>();// / acc9.num;
    b_out = mAcc9.H.topRightCorner<8,1>();// / acc9.num;
    H_out_sc = mAcc9SC.H.topLeftCorner<8,8>();// / acc9.num;
    b_out_sc = mAcc9SC.H.topRightCorner<8,1>();// / acc9.num;



    H_out(0,0) += alphaOpt*npts;
    H_out(1,1) += alphaOpt*npts;
    H_out(2,2) += alphaOpt*npts;

    Vector3f tlog = SE3(camera.getRotationMatrix(), camera.getTranslation()).log().head<3>().cast<float>(); // todo : check this
    b_out[0] += tlog[0]*alphaOpt*npts;
    b_out[1] += tlog[1]*alphaOpt*npts;
    b_out[2] += tlog[2]*alphaOpt*npts;

    // assertThrow(numInliers > 0, "No inliers ?");

    return Vector3f(E.A, alphaEnergy ,E.num);
}

void CML::Optimization::DSOInitializer::propagateUp(int srcLvl) {
    assert(srcLvl+1<mNumPyramidLevel);

    // set to zero.
    for(size_t i = 0; i < mPoints[srcLvl + 1].size(); i++) {
        auto &parentData = mPoints[srcLvl + 1][i];
        parentData.iR=0;
        parentData.iRSumNum=0;
    }

    for(size_t i = 0; i < mPoints[srcLvl].size(); i++)
    {
        auto &pointData = mPoints[srcLvl][i];
        if(!pointData.isGood) continue;

        auto &parentData = mPoints[srcLvl + 1][pointData.parent];
        parentData.iR += pointData.iR * pointData.lastHessian;
        parentData.iRSumNum += pointData.lastHessian;
    }

    for(size_t i = 0; i < mPoints[srcLvl + 1].size(); i++)
    {
        auto &parentData = mPoints[srcLvl + 1][i];
        if(parentData.iRSumNum > 0)
        {
            parentData.idepth = parentData.iR = (parentData.iR / parentData.iRSumNum);
            parentData.isGood = true;
        }
    }

    optReg(srcLvl+1);
}

void CML::Optimization::DSOInitializer::propagateDown(int srcLvl) {
    assert(srcLvl>0);
    // set idepth of target

    int dstLvl = srcLvl - 1;

    for(size_t i = 0; i < mPoints[dstLvl].size(); i++) {
        auto &dstPointData = mPoints[dstLvl][i];
        auto &srcPointData = mPoints[srcLvl][dstPointData.parent];

        if(!srcPointData.isGood || srcPointData.lastHessian < 0.1) continue;
        if(!dstPointData.isGood)
        {
            dstPointData.iR = dstPointData.idepth = dstPointData.idepth_new = srcPointData.iR;
            dstPointData.isGood=true;
            dstPointData.lastHessian=0;
        }
        else
        {
            float newiR = (dstPointData.iR * dstPointData.lastHessian * SCALEFACTOR + srcPointData.iR * srcPointData.lastHessian) / (dstPointData.lastHessian * SCALEFACTOR + srcPointData.lastHessian);
            dstPointData.iR = dstPointData.idepth = dstPointData.idepth_new = newiR;
        }
    }
    optReg(srcLvl-1);
}

CML::Vector3f CML::Optimization::DSOInitializer::calcEC(int lvl) {
    if(!mSnapped) return Vector3f(0,0,mPoints[lvl].size());
    dso::AccumulatorX<2> E;
    E.initialize();

    for(size_t i = 0; i < mPoints[lvl].size(); i++) {
        auto &pointData = mPoints[lvl][i];
        if(!pointData.isGood_new) continue;
        float rOld = (pointData.idepth - pointData.iR);
        float rNew = (pointData.idepth_new - pointData.iR);
        E.updateNoWeight(Vector2f(rOld*rOld,rNew*rNew));

        //printf("%f %f %f!\n", point->idepth, point->idepth_new, point->iR);
    }
    E.finish();

    //printf("ER: %f %f %f!\n", couplingWeight*E.A1m[0], couplingWeight*E.A1m[1], (float)E.num.numIn1m);
    return Vector3f(mCouplingWeight*E.A1m[0], mCouplingWeight*E.A1m[1], E.num);
}

void CML::Optimization::DSOInitializer::optReg(int lvl) {

    if(!mSnapped)
    {
        for(size_t i = 0; i < mPoints[lvl].size(); i++) {
            auto &pointData = mPoints[lvl][i];
            pointData.iR = pointData.initialiR;
        }
        return;
    }

    for(size_t i = 0; i < mPoints[lvl].size(); i++) {
        auto &pointData = mPoints[lvl][i];
        if(!pointData.isGood) continue;

        List<float> idnn; idnn.reserve(10);
        for(int j=0;j<10;j++)
        {
            if(pointData.neighbours[j] == -1) continue;
            auto &pointDataOther = mPoints[lvl][j];
            if(!pointDataOther.isGood) continue;
            idnn.emplace_back(pointDataOther.iR);
        }

        if(idnn.size() > 2)
        {
            pointData.iR = (1 - mRegWeight) * pointData.idepth + mRegWeight * median(idnn);
        }
        pointData.iR = (1 - mNNWeight) * pointData.iR + mNNWeight * pointData.initialiR;

    }

}

void CML::Optimization::DSOInitializer::resetPoints(int lvl) {
    for(size_t i = 0; i < mPoints[lvl].size(); i++)
    {
        auto &pointData = mPoints[lvl][i];
        pointData.energy.setZero();
        pointData.idepth_new = pointData.idepth;


        if(lvl == mNumPyramidLevel - 1 && !pointData.isGood)
        {
            float snd=0, sn=0;
            for(int n = 0;n<10;n++)
            {
                if (pointData.neighbours[n] == -1) continue;

                auto &neighboursData = mPoints[lvl][pointData.neighbours[n]];

                if(!neighboursData.isGood) continue;
                snd += neighboursData.iR;
                sn += 1;
            }

            if(sn > 0)
            {
                pointData.isGood=true;
                pointData.iR = pointData.idepth = pointData.idepth_new = snd/sn;
            }
        }
    }
}

void CML::Optimization::DSOInitializer::doStep(int lvl, float lambda, Vector8f inc) {
    const float maxPixelStep = 0.25;
    const float idMaxStep = 1e10;

    List<float> allSteps;

    for(size_t i = 0; i < mPoints[lvl].size(); i++)
    {
        auto &pointData = mPoints[lvl][i];
        if(!pointData.isGood) continue;

        float b = mJbBuffer[i][8] + mJbBuffer[i].head<8>().dot(inc);
        float step = - b * mJbBuffer[i][9] / (1+lambda);

        float maxstep = maxPixelStep*pointData.maxstep;
        if(maxstep > idMaxStep) maxstep=idMaxStep;

        if(step >  maxstep) step = maxstep;
        if(step < -maxstep) step = -maxstep;

        float newIdepth = pointData.idepth + step;
        if(newIdepth < 1e-3 ) newIdepth = 1e-3;
        if(newIdepth > 50) newIdepth = 50;
        pointData.idepth_new = newIdepth;

        allSteps.emplace_back(step);

    }


}

void CML::Optimization::DSOInitializer::applyStep(int lvl) {

    for(size_t i = 0; i < mPoints[lvl].size(); i++)
    {
        auto &pointData = mPoints[lvl][i];
        if(!pointData.isGood)
        {
            pointData.idepth = pointData.idepth_new = pointData.iR;
            continue;
        }

        pointData.energy = pointData.energy_new;
        pointData.isGood = pointData.isGood_new;
        pointData.idepth = pointData.idepth_new;
        pointData.lastHessian = pointData.lastHessian_new;
    }

    std::swap(mJbBuffer, mJbBuffer_new);
}

void CML::Optimization::DSOInitializer::makeNN() {
    const float NNDistFactor=0.05;

    // build indices
    PointGrid<DSOInitializerPoint> *indexes[mNumPyramidLevel];
    for(int i = 0; i < mNumPyramidLevel; i++)
    {
        indexes[i] = new PointGrid<DSOInitializerPoint>(mPoints[i], Vector2i(0,0), Vector2i(mReference->getWidth(i), mReference->getHeight(i)));
    }

    const int nn=10;

    // find NN & parents
    for(int lvl = 0; lvl < mNumPyramidLevel;lvl++)
    {
        for(size_t i = 0; i < mPoints[lvl].size(); i++)
        {
            auto &pointData = mPoints[lvl][i];
            Vector2f pt = pointData.p;
            auto nearestNeighbors = indexes[lvl]->searchInRadiusNum(pt, nn);

            int myidx=0;
            float sumDF = 0;
            for(int k = 0; k < nn; k++)
            {
                pointData.neighbours[myidx] = nearestNeighbors[k].index;
                float df = expf(-nearestNeighbors[k].distance * NNDistFactor);
                sumDF += df;
                pointData.neighboursDist[myidx] = df;
                myidx++;
            }

            for(int k=0;k<nn;k++) {
                pointData.neighboursDist[k] *= 10 / sumDF;
            }

            if(lvl < mNumPyramidLevel - 1)
            {
                pt = pt / SCALEFACTOR;
                auto nearestNeighbors1 = indexes[lvl+1]->searchInRadiusNum(pt, 1);

                pointData.parent = nearestNeighbors1[0].index;
                pointData.parentDist = expf(-nearestNeighbors1[0].distance*NNDistFactor);

            }
            else
            {
                pointData.parent = -1;
                pointData.parentDist = -1;
            }
        }
    }

    for(int i=0;i<mNumPyramidLevel;i++) {
        delete indexes[i];
    }
}

inline CML::Vector3f makeRainbow3B(float id)
{
    if(!(id > 0))
        return CML::Vector3f(1,1,1);

    int icP = id;
    float ifP = id-icP;
    icP = icP%3;

    if(icP == 0) return CML::Vector3f(255*(1-ifP), 255*ifP,     0) / 255.0f;
    if(icP == 1) return CML::Vector3f(0,           255*(1-ifP), 255*ifP) / 255.0f;
    if(icP == 2) return CML::Vector3f(255*ifP,     0,           255*(1-ifP)) / 255.0f;
    return CML::Vector3f(255,255,255) / 255.0f;
}

void CML::Optimization::DSOInitializer::viewOnCapture(DrawBoard &drawBoard, PFrame frame) {
    LockGuard lg(mPointsMutex);

    if (mPoints.empty()) {
        return;
    }

    float nid = 0, sid = 0;
    for(auto &point : mPoints[0])
    {
        if(point.isGood)
        {
            nid++;
            sid += point.iR;
        }
    }
    float fac = nid / sid;


    for(auto &point : mPoints[0])
    {
        if(!point.isGood) {
            drawBoard.color(0, 0, 0);
            drawBoard.pointSize(1);
            drawBoard.point(point.p);
        }
        else {

            drawBoard.color(makeRainbow3B(point.iR * fac));
            drawBoard.pointSize(1);
            drawBoard.point(point.p);
        }
    }
}

void CML::Optimization::DSOInitializer::viewOnReconstruction(DrawBoard &drawBoard) {
    LockGuard lg(mPointsMutex);

    if (mPoints.empty()) {
        return;
    }

    float nid = 0, sid = 0;
    for(auto &point : mPoints[0])
    {
        if(point.isGood)
        {
            nid++;
            sid += point.iR;
        }
    }
    float fac = nid / sid;


    for(auto &point : mPoints[0])
    {
        if(!point.isGood) {
            drawBoard.color(0, 0, 0);
            drawBoard.pointSize(1);
        }
        else {

            drawBoard.color(makeRainbow3B(point.iR * fac));
            drawBoard.pointSize(1);
        }

        UndistortedVector2d undistorted = mReference->undistort(DistortedVector2d(Vector2(point.x(), point.y())), 0);
        Vector3 pt = WorldPoint::fromInverseDepth(point.idepth_new, undistorted, mReference->getCamera()).relative(mCurrentCamera);

        drawBoard.point((Vector3f)pt.cast<float>());

    }
}