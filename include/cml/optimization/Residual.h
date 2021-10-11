#ifndef CML_RESIDUAL_H
#define CML_RESIDUAL_H

#include <cml/config.h>
#include <cml/map/Frame.h>
#include <cml/map/MapObject.h>
#include <cml/image/Array2D.h>

namespace CML {

    class ReprojectionError {

    public:
        ReprojectionError(PFrame frame, PPoint mapPoint) : mFrame(frame), mMapPoint(mapPoint) {
            mGt = mFrame->undistort(mFrame->getFeaturePoint(mapPoint).value().point(0), 0);
        }

        bool error(scalar_t &residual) const {
            Vector3 transformation = mFrame->getCamera().transform(mMapPoint->getWorldCoordinate().absolute());
            Vector2 projection = transformation.hnormalized();
            Vector2 difference = projection - mGt;
            scalar_t squaredNorm = difference.squaredNorm();
            residual = squaredNorm;
            return true;
        }

        bool trialError(scalar_t &residual, const Camera &trialCamera, const Vector3 &trialWorldCoordinate) const {
            Vector3 transformation = trialCamera.transform(trialWorldCoordinate);
            Vector2 projection = transformation.hnormalized();
            Vector2 difference = projection - mGt;
            scalar_t squaredNorm = difference.squaredNorm();
            residual = squaredNorm;
            return true;
        }

        template <typename JT, typename JR, typename JP>
        bool jacobian(scalar_t &residual, const Camera &trialCamera, const Vector3 &trialWorldCoordinate, JT &translationJacobian, JR &rotationJacobian, JP &pointJacobian) const {
            Vector3 transformation = trialCamera.transform(trialWorldCoordinate);
            Vector2 projection = transformation.hnormalized();
            Vector2 difference = projection - mGt;
            scalar_t squaredNorm = difference.squaredNorm();

            residual = squaredNorm;

            if (translationJacobian != nullptr || pointJacobian != nullptr) for (int i = 0; i < 3; i++) {
                double v = Derivative::squaredNorm(difference, trialCamera.derivativeOfProjectWrtTranslation(transformation, i));
                // assertThrow(std::isfinite(v), "Not finite residual");
                // if (!std::isfinite(v)) {
                //    return false;
                // }
                if (translationJacobian != nullptr) translationJacobian[i] = v;
                if (pointJacobian != nullptr) pointJacobian[i] = -v;
            }

            if (rotationJacobian != nullptr) for (int i = 0; i < Quaternion::N; i++) {
                rotationJacobian[i] = Derivative::squaredNorm(difference, trialCamera.derivativeOfProjectWrtRotation(trialWorldCoordinate, transformation, i));
                // assertThrow(std::isfinite(rotationJacobian[i]), "Not finite residual");
                // if (!std::isfinite(rotationJacobian[i])) {
                //     return false;
                // }
            }

            return true;
        }

        PFrame getFrame() const {
            return mFrame;
        }

        PPoint getMapPoint() const {
            return mMapPoint;
        }


    private:
        PFrame mFrame;
        PPoint mMapPoint;
        UndistortedVector2d mGt;

    };

    class PhotometricError {

    public:
        PhotometricError(PFrame frame, PPoint mapPoint)
        : mRefFrame(mapPoint->getReferenceFrame()), mCurFrame(frame), mMapPoint(mapPoint), mRefCorner(mMapPoint->getReferenceCorner()) {
        }

        bool error(scalar_t &residual, const Vector2 &shift, int level) const {
            DistortedVector2d refcorner_Distorted = DistortedVector2d(mRefCorner.point(level) + shift.cast<scalar_t>());

            // First, unproject the point from reference camera
            UndistortedVector2d refcorner = mRefFrame->undistort(refcorner_Distorted, level);
            Vector3 refp = (Vector3) refcorner.homogeneous() / mMapPoint->getReferenceInverseDepth();
            Vector3 unrotatedrefp = mRefFrame->getCamera().getRotationMatrix() * refp;
            Vector3 worldp = unrotatedrefp + mRefFrame->getCamera().getTranslation();

            // Then, project the point on the current camera
            Vector3 translatedcurp = worldp - mCurFrame->getCamera().getTranslation();
            Vector3 transformedcurp = mCurFrame->getCamera().getRotationMatrix().transpose() * translatedcurp;
            UndistortedVector2d projectedcurp = UndistortedVector2d(transformedcurp.hnormalized());
            DistortedVector2d projectedcurp_Distorted = mCurFrame->distort(projectedcurp, level);

            // Check that the projection is finite
            if (!std::isfinite(projectedcurp_Distorted.x()) || !std::isfinite(projectedcurp_Distorted.y())) {
                return false;
            }

            // Check that the projection is inside the current frame, with a padding of 3, to allow the computation of the derivative on the image
            if (!mCurFrame->isInside(projectedcurp_Distorted, level, 3)) {
                return false;
            }

            scalar_t curColor = mCurFrame->getCaptureFrame().getGrayImage(level).interpolate(projectedcurp_Distorted);
            scalar_t refColor = mMapPoint->getGrayPatch(shift.x(), shift.y(), level);

            ExposureTransition exposureTransition = mRefFrame->getExposure().to(mCurFrame->getExposure());

            scalar_t curRealColor = curColor;
            scalar_t refRealColor = exposureTransition(refColor);

            scalar_t difference = curRealColor - refRealColor;

            residual = fabs(difference);

            return true;
        }

        bool trialError(scalar_t &residual, const Vector2 &shift, int level, const Camera &trialReference, const Exposure &trialExposureRef, const Camera &trialCurrent, const Exposure &trialExposureCur, double trialDepth) const {
            DistortedVector2d refcorner_Distorted = DistortedVector2d(mRefCorner.point(level) + shift.cast<scalar_t>());

            // First, unproject the point from reference camera
            UndistortedVector2d refcorner = mRefFrame->undistort(refcorner_Distorted, level);
            Vector3 refp = (Vector3) refcorner.homogeneous() * trialDepth;
            Vector3 unrotatedrefp = trialReference.getRotationMatrix() * refp;
            Vector3 worldp = unrotatedrefp + trialReference.getTranslation();

            // Then, project the point on the current camera
            Vector3 translatedcurp = worldp - trialCurrent.getTranslation();
            Vector3 transformedcurp = trialCurrent.getRotationMatrix().transpose() * translatedcurp;
            UndistortedVector2d projectedcurp = UndistortedVector2d(transformedcurp.hnormalized());
            DistortedVector2d projectedcurp_Distorted = mCurFrame->distort(projectedcurp, level);

            // Check that the projection is finite
            if (!std::isfinite(projectedcurp_Distorted.x()) || !std::isfinite(projectedcurp_Distorted.y())) {
                return false;
            }

            // Check that the projection is inside the current frame, with a padding of 3, to allow the computation of the derivative on the image
            if (!mCurFrame->isInside(projectedcurp_Distorted, level, 3)) {
                return false;
            }

            scalar_t curColor = mCurFrame->getCaptureFrame().getGrayImage(level).interpolate(projectedcurp_Distorted);
            scalar_t refColor = mMapPoint->getGrayPatch(shift.x(), shift.y(), level);

            ExposureTransition exposureTransition = trialExposureRef.to(trialExposureCur);

            scalar_t curRealColor = curColor;
            scalar_t refRealColor = exposureTransition(refColor);

            scalar_t difference = curRealColor - refRealColor;

            residual = fabs(difference);

            return true;
        }

        bool error(scalar_t &residual, const Pattern &pattern, int level) const {
            residual = 0;
            for (int i = 0; i < pattern.size(); i++) {
                scalar_t currentResidual;
                if (!error(currentResidual, pattern[i], level)) {
                    return false;
                }
                residual += currentResidual;
            }
            return true;
        }

        bool trialError(scalar_t &residual, const Pattern &pattern, int level, const Camera &trialReference, const Exposure &trialExposureRef, const Camera &trialCurrent, const Exposure &trialExposureCur, double trialDepth) const {
            residual = 0;
            for (int i = 0; i < pattern.size(); i++) {
                scalar_t currentResidual;
                if (!trialError(currentResidual, pattern[i], level, trialReference, trialExposureRef, trialCurrent, trialExposureCur, trialDepth)) {
                    return false;
                }
                residual += currentResidual;
            }
            return true;
        }

        template <typename JT, typename JR, typename JE>
        bool jacobian(const Vector2 &shift, int level, scalar_t &residual, const Camera &trialReference, const Exposure &trialExposureRef, const Camera &trialCurrent, const Exposure &trialExposureCur, double trialDepth, JT &refTranslationJacobian, JR &refRotationJacobian, JE &refExposureJacobian,  JT &curTranslationJacobian, JR &curRotationJacobian, JE &curExposureJacobian, scalar_t *pointJacobian) const {

            DistortedVector2d refcorner_Distorted = DistortedVector2d(mRefCorner.point(level) + shift.cast<scalar_t>());

            // First, unproject the point from reference camera
            UndistortedVector2d refcorner = mRefFrame->undistort(refcorner_Distorted, level);
            Vector3 refp = (Vector3) refcorner.homogeneous() * trialDepth;
            Vector3 unrotatedrefp = trialReference.getRotationMatrix() * refp;
            Vector3 worldp = unrotatedrefp + trialReference.getTranslation();

            // Then, project the point on the current camera
            Vector3 translatedcurp = worldp - trialCurrent.getTranslation();
            Vector3 transformedcurp = trialCurrent.getRotationMatrix().transpose() * translatedcurp;
            UndistortedVector2d projectedcurp = UndistortedVector2d(transformedcurp.hnormalized());
            DistortedVector2d projectedcurp_Distorted = mCurFrame->distort(projectedcurp, level);

            // Check that the projection is finite
            if (!std::isfinite(projectedcurp_Distorted.x()) || !std::isfinite(projectedcurp_Distorted.y())) {
                return false;
            }

            // Check that the projection is inside the current frame, with a padding of 3, to allow the computation of the derivative on the image
            if (!mCurFrame->isInside(projectedcurp_Distorted, level, 3)) {
                return false;
            }

            auto curColorWithGradient = mCurFrame->getCaptureFrame().getDerivativeImage(level).interpolate(projectedcurp_Distorted);

            scalar_t curColor = curColorWithGradient.c();
            Vector2 curColorGradient = curColorWithGradient.gradient();
            scalar_t refColor = mMapPoint->getGrayPatch(shift.x(), shift.y(), level); // TODO : Use this for a faster access
            // scalar_t refColor = mRefFrame->getCaptureFrame().getGrayImage(level).template interpolate<scalar_t>(refcorner_Distorted);

            ExposureTransition exposureTransition = trialExposureRef.to(trialExposureCur);

            scalar_t curRealColor = curColor;
            scalar_t refRealColor = exposureTransition(refColor);

            scalar_t difference = curRealColor - refRealColor;

            residual = difference;

            if (refTranslationJacobian != nullptr) {
                for (int i = 0; i < 3; i++) {
                    Vector3 reft_wrt_refti = Vector3::Zero();
                    reft_wrt_refti(i) = 1;

                    Vector2 projectedcurp_wrt_refti = mCurFrame->getK(level).block(0, 0, 2, 2) * Derivative::hnormalized(transformedcurp, trialCurrent.getRotationMatrix().transpose() * reft_wrt_refti);
                    refTranslationJacobian[i] = Derivative::interpolate(curColorGradient, projectedcurp_wrt_refti);
                }
            }

            if (curTranslationJacobian != nullptr) {
                for (int i = 0; i < 3; i++) {
                    Vector3 minusCurt_wrtcurti = Vector3::Zero();
                    minusCurt_wrtcurti(i) = -1;

                    Vector2 projectedcurp_wrt_curti = mCurFrame->getK(level).block(0, 0, 2, 2) * Derivative::hnormalized(transformedcurp, trialCurrent.getRotationMatrix().transpose() * minusCurt_wrtcurti);
                    curTranslationJacobian[i] = Derivative::interpolate(curColorGradient, projectedcurp_wrt_curti);
                }
            }

            if (refRotationJacobian != nullptr) {
                for (int i = 0; i < Quaternion::N; i++) {
                    Vector2 projectedcurp_wrt_refai = mCurFrame->getK(level).block(0, 0, 2, 2) * Derivative::hnormalized(transformedcurp, trialCurrent.getRotationMatrix().transpose() * (Quaternion(trialReference.getRotationMatrix()).matrixDerivative(i) * refp));
                    refRotationJacobian[i] = Derivative::interpolate(curColorGradient, projectedcurp_wrt_refai);
                }
            }

            if (curRotationJacobian != nullptr) {
                for (int i = 0; i < Quaternion::N; i++) {
                    Vector2 projectedcurp_wrt_curai =  mCurFrame->getK(level).block(0, 0, 2, 2) * Derivative::hnormalized(transformedcurp, Quaternion(trialCurrent.getRotationMatrix()).inverseMatrixDerivative(i) * translatedcurp);
                    curRotationJacobian[i] = Derivative::interpolate(curColorGradient, projectedcurp_wrt_curai);
                }
            }

            /*if (refExposureJacobian != nullptr) {
                for (int i = 0; i < 2; i++) {
                    scalar_t refColor_wrt_e = -trialExposureRef.unapplyDerivative(refColor, i);
                    refExposureJacobian[i] = refColor_wrt_e;
                }
            }

            if (curExposureJacobian != nullptr) {
                for (int i = 0; i < 2; i++) {
                    scalar_t curColor_wrt_e = trialExposureCur.unapplyDerivative(curColor, i);
                    curExposureJacobian[i] = curColor_wrt_e;
                }
            }*/ // TODO !!!

            if (pointJacobian != nullptr) {

                Vector3 refp_wrt_depth = refcorner.homogeneous();
                Vector3 unrotatedrefp_wrt_depth = trialReference.getRotationMatrix() * refp_wrt_depth;

                Vector3 transformedcurp_wrt_depth = trialCurrent.getRotationMatrix().transpose() * unrotatedrefp_wrt_depth;
                Vector2 projectedcurp_wrt_depth = mCurFrame->getK(level).block(0, 0, 2, 2) * Derivative::hnormalized(transformedcurp,transformedcurp_wrt_depth);

                pointJacobian[0] = Derivative::interpolate(curColorGradient, projectedcurp_wrt_depth);

            }

            return true;

        }

        PFrame getReferenceFrame() const {
            return mRefFrame;
        }

        PFrame getCurrentFrame() const {
            return mCurFrame;
        }

        PPoint getMapPoint() const {
            return mMapPoint;
        }

    private:
        PFrame mRefFrame, mCurFrame;
        PPoint mMapPoint;
        Corner mRefCorner;

    };


}

#endif