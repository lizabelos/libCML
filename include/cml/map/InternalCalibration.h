//
// Created by tbelos on 19/04/19.
//

#ifndef CML_INTERNALCALIBRATION_H
#define CML_INTERNALCALIBRATION_H

#include <string>
#include <memory>
#include <iostream>
#include "cml/config.h"
#include "cml/utils/Logger.h"
#include "cml/image/Array2D.h"

#define CML_INTERNALCALIBRATION_LEVELS 16

namespace CML {

    class Undistorter {

    public:
        virtual Vector2d distort(const Vector2d &input) const = 0;
        virtual Vector2d undistort(const Vector2d &input) const = 0;

        virtual Vector2f distort(const Vector2f &input) const = 0;
        virtual Vector2f undistort(const Vector2f &input) const = 0;

        virtual std::string toString() const = 0;

    };

    class PinholeUndistorter : public Undistorter {

    public:
        PinholeUndistorter() : mF(Vector2(1, 1)), mC(Vector2(0.5, 0.5)) {
            mFinv(0) = 1.0 / mF(0);
            mFinv(1) = 1.0 / mF(1);
            mCinv(0) = 1.0 / mC(0);
            mCinv(1) = 1.0 / mC(1);
        }

        PinholeUndistorter(Vector2 f, Vector2 c) : mF(f), mC(c) {
            mFinv(0) = 1.0 / mF(0);
            mFinv(1) = 1.0 / mF(1);
            mCinv(0) = 1.0 / mC(0);
            mCinv(1) = 1.0 / mC(1);
        }

        PinholeUndistorter(Vector4 parameters) : mF(parameters.head<2>()), mC(parameters.tail<2>()) {
            mFinv(0) = 1.0 / mF(0);
            mFinv(1) = 1.0 / mF(1);
            mCinv(0) = 1.0 / mC(0);
            mCinv(1) = 1.0 / mC(1);
        }

        EIGEN_STRONG_INLINE Vector2d distort(const Vector2d &input) const override {
            return Vector2d(input.x() * mF.x() + mC.x(), input.y() * mF.y() + mC.y());
        }

        EIGEN_STRONG_INLINE Vector2d undistort(const Vector2d &input) const override {
            return Vector2d((input.x() - mC.x()) * mFinv.x(), (input.y() - mC.y()) * mFinv.y());
        }

        EIGEN_STRONG_INLINE Vector2f distort(const Vector2f &input) const override {
            return Vector2f(input.x() * (float)mF.x() + (float)mC.x(), input.y() * (float)mF.y() + (float)mC.y());
        }

        EIGEN_STRONG_INLINE Vector2f undistort(const Vector2f &input) const override {
            return Vector2f((input.x() - (float)mC.x()) * (float)mFinv.x(), (input.y() - (float)mC.y()) * (float)mFinv.y());
        }

        EIGEN_STRONG_INLINE Matrix33 getK() const {
            Matrix33 K = Matrix33::Identity();
            K(0, 0) = mF.x();
            K(1, 1) = mF.y();
            K(0, 2) = mC.x();
            K(1, 2) = mC.y();
            return K;
        }

        EIGEN_STRONG_INLINE Vector4 getParameters() const {
            return Vector4(mF.x(), mF.y(), mC.x(), mC.y());
        }

        EIGEN_STRONG_INLINE PinholeUndistorter level(int lvl) const {
            if (lvl == 0) return *this;
            scalar_t d = (int)(1 << lvl);
            return PinholeUndistorter(Vector2(
                    mF(0) / d,
                    mF(1) / d
                    ), Vector2(
                    (mC(0) + 0.5) / d - 0.5,
                    (mC(1) + 0.5) / d - 0.5
                    ));
        }

        EIGEN_STRONG_INLINE PinholeUndistorter scale(Vector2 v) {
            return PinholeUndistorter(Vector2(mF(0) * v(0), mF(1) * v(1)), Vector2(mC(0) * v(0), mC(1) * v(1)));
        }

        EIGEN_STRONG_INLINE PinholeUndistorter scaleAndRecenter(Vector2 v, Vector2 c) {
            return PinholeUndistorter(Vector2(mF(0) * v(0), mF(1) * v(1)), Vector2(mC(0) * v(0) + c(0), mC(1) * v(1) + c(1)));
        }

        std::string toString() const final {
            return "[PinholeUndistorter](" + std::to_string(mF.x()) + " ; " + std::to_string(mF.y()) + " ; " + std::to_string(mC.x()) + " ; " + std::to_string(mC.y()) + ")";
        }

    private:
        Vector2 mF, mC, mFinv, mCinv;

    };

    class RadtanUndistorter : public Undistorter {

    public:
        RadtanUndistorter(scalar_t k1, scalar_t k2, scalar_t p1, scalar_t p2) : mP1(p1), mP2(p2), mK1(k1), mK2(k2) {

        }

        EIGEN_STRONG_INLINE Vector2d distort(const Vector2d &input) const override {
            scalar_t mx2_u = input.x() * input.x();
            scalar_t my2_u = input.y() * input.y();
            scalar_t mxy_u = input.x() * input.y();
            scalar_t rho2_u = mx2_u+my2_u;
            scalar_t rad_dist_u = mK1 * rho2_u + mK2 * rho2_u * rho2_u;
            return Vector2d(
                    input.x() + input.x() * rad_dist_u + 2.0 * mP1 * mxy_u + mP2 * (rho2_u + 2.0 * mx2_u),
                    input.y() + input.y() * rad_dist_u + 2.0 * mP2 * mxy_u + mP1 * (rho2_u + 2.0 * my2_u)
                    );
        }

        EIGEN_STRONG_INLINE Vector2d undistort(const Vector2d &input) const override {
            scalar_t r2 = input.x() * input.x() + input.y() * input.y();
            scalar_t r4 = r2 * r2;

            return Vector2d(
                    input.x() * (1.0 + scalar_t(mK1) * r2 + scalar_t(mK2) * r4) + 2.0 * scalar_t(mP1) * input.x() * input.y() + scalar_t(mP2) * (r2 + 2.0 * input.x() * input.x()),
                    input.y() * (1.0 + scalar_t(mK1) * r2 + scalar_t(mK2) * r4) + 2.0 * scalar_t(mP2) * input.x() * input.y() + scalar_t(mP1) * (r2 + 2.0 * input.y() * input.y())
                    );
        }

        EIGEN_STRONG_INLINE Vector2f distort(const Vector2f &input) const override {
            float mx2_u = input.x() * input.x();
            float my2_u = input.y() * input.y();
            float mxy_u = input.x() * input.y();
            float rho2_u = mx2_u+my2_u;
            float rad_dist_u = mK1 * rho2_u + mK2 * rho2_u * rho2_u;
            return Vector2f(
                    input.x() + input.x() * rad_dist_u + 2.0 * mP1 * mxy_u + mP2 * (rho2_u + 2.0 * mx2_u),
                    input.y() + input.y() * rad_dist_u + 2.0 * mP2 * mxy_u + mP1 * (rho2_u + 2.0 * my2_u)
            );
        }

        EIGEN_STRONG_INLINE Vector2f undistort(const Vector2f &input) const override {
            scalar_t r2 = input.x() * input.x() + input.y() * input.y();
            scalar_t r4 = r2 * r2;

            return Vector2f(
                    input.x() * (1.0 + scalar_t(mK1) * r2 + scalar_t(mK2) * r4) + 2.0 * scalar_t(mP1) * input.x() * input.y() + scalar_t(mP2) * (r2 + 2.0 * input.x() * input.x()),
                    input.y() * (1.0 + scalar_t(mK1) * r2 + scalar_t(mK2) * r4) + 2.0 * scalar_t(mP2) * input.x() * input.y() + scalar_t(mP1) * (r2 + 2.0 * input.y() * input.y())
            );
        }

        std::string toString() const final {
            return "[RadtanUndistorter](" + std::to_string(mP1) + " ; " + std::to_string(mP2) + " ; " +  std::to_string(mK1) + " ; " + std::to_string(mK2) + ")";
        }


    private:
        scalar_t mP1, mP2, mK1, mK2;

    };

    class FOVUndistorter : public Undistorter {

    public:
        FOVUndistorter(scalar_t dist) : mDist(dist), mD2t(2.0 * tan(dist / 2.0)) {

        }

        EIGEN_STRONG_INLINE Vector2d distort(const Vector2d &input) const override {
            scalar_t r = sqrt(input.x() * input.x() + input.y() * input.y());
            scalar_t fac = (r == 0 || mDist == 0) ? 1 : atan(r * mD2t)/(mDist * r);
            return input * fac;
        }

        EIGEN_STRONG_INLINE Vector2d undistort(const Vector2d &input) const override {
            assertThrow(false, "Not implemented");
            return Vector2d(0, 0);
            /*scalar_t ru = sqrt(input.x() * input.x() + input.y() * input.y());
            scalar_t rd = (1.0 / mDist) * atan(ru * mD2t);
            return input * (rd / ru);*/
        }

        EIGEN_STRONG_INLINE Vector2f distort(const Vector2f &input) const override {
            float r = sqrtf(input.x() * input.x() + input.y() * input.y());
            float fac = (r == 0 || mDist == 0) ? 1 : atanf(r * (float)mD2t)/((float)mDist * r);
            return input * fac;
        }

        EIGEN_STRONG_INLINE Vector2f undistort(const Vector2f &input) const override {
            assertThrow(false, "Not implemented");
            return Vector2f(0, 0);
            /*scalar_t ru = sqrt(input.x() * input.x() + input.y() * input.y());
            scalar_t rd = (1.0 / mDist) * atan(ru * mD2t);
            return input * (rd / ru);*/
        }

        std::string toString() const final {
            return "[FOVUndistorter](" + std::to_string(mDist) + ")";
        }

    private:
        const scalar_t mDist, mD2t;

    };

    class FishEye10_5_5 : public Undistorter {

    public:
        FishEye10_5_5(const List<scalar_t> &R, const Vector2 &P, const Vector2 &t) : mR(R), mP(P), mt(t) {

        }

        EIGEN_STRONG_INLINE Vector2d distort(const Vector2d &input) const final {
            return computeDpol(input.cast<scalar_t>()).cast<double>();
        }

        EIGEN_STRONG_INLINE Vector2f distort(const Vector2f &input) const final {
            return computeDpol(input.cast<scalar_t>()).cast<float>();
        }

        EIGEN_STRONG_INLINE Vector2d undistort(const Vector2d &input) const final {
            assertThrow(false, "Not implemented");
            return Vector2d(0, 0);
        }

        EIGEN_STRONG_INLINE Vector2f undistort(const Vector2f &input) const final {
            assertThrow(false, "Not implemented");
            return Vector2f(0, 0);
        }

        std::string toString() const final {
            return "[FishEye10_5_5]()";
        }

    protected:
        EIGEN_STRONG_INLINE scalar_t epsilon(scalar_t v) const {
            if (meEuilinear) {
                return v;
            } else {
                return 2 * sin(v / 2);
            }
        }

        EIGEN_STRONG_INLINE Vector2 computeDpol(const Vector2 &AB) const {
            scalar_t R = AB.norm();
            scalar_t lambda = epsilon(atan(R)) / R;
            Vector2 ab = AB * lambda;
            scalar_t a = ab(0);
            scalar_t b = ab(1);
            scalar_t p = ab.norm();

            Vector2 result;

            // Left
            scalar_t left = 1;
            int power = 2;
            for (int i = 0; i < mR.size(); i++) {
                left += pow(mR[i], power);
                power = power * 2;
            }
            result = ab * left;

            // Mid
            result.x() += mP(0) * (p*p + 2*a*a) + 2 * mP(1) * a * b;
            result.y() += 2 * mP(0) * a * b + mP(1) * (p*p + 2*b*b);

            // Right
            result.x() += mt(0) * a + mt(1) * b;
            result.y() += mt(1) * a;

            return result;
        }


    private:
        List<scalar_t> mR;
        Vector2 mP;
        Vector2 mt;
        bool meEuilinear = true;

    };

    class InternalCalibration {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        InternalCalibration() = default;

        InternalCalibration(PinholeUndistorter originalPinhole, Vector2 originalSize) {
            mOriginalPinhole = originalPinhole;
            mOriginalSize = originalSize;
            mNewPinhole = mOriginalPinhole;
            mNewSize = mOriginalSize;
            mPreundistorter = nullptr;
            mDefaultParameter = false;
            computeLevels();
            computeUndistortMap();
        }

        InternalCalibration(PinholeUndistorter originalPinhole, Vector2 originalSize, Undistorter *preundistorter) {
            mOriginalPinhole = originalPinhole;
            mOriginalSize = originalSize;
            mNewPinhole = mOriginalPinhole;
            mNewSize = mOriginalSize;
            mPreundistorter = preundistorter;
            mDefaultParameter = false;
            computeLevels();
            computeUndistortMap();
        }

        InternalCalibration(PinholeUndistorter originalPinhole, Vector2 originalSize, Undistorter *preundistorter, PinholeUndistorter newPinhole, Vector2 newSize) {
            mOriginalPinhole = originalPinhole;
            mOriginalSize = originalSize;
            mNewPinhole = newPinhole;
            mNewSize = newSize;
            mPreundistorter = preundistorter;
            mDefaultParameter = false;
            computeLevels();
            computeUndistortMap();
        }

        InternalCalibration(PinholeUndistorter originalPinhole, Vector2 originalSize, Undistorter *preundistorter, PinholeUndistorter newPinhole, Vector2 newSize, const Array2D<Vector2f> &undistortMap) {
            mOriginalPinhole = originalPinhole;
            mOriginalSize = originalSize;
            mNewPinhole = newPinhole;
            mNewSize = newSize;
            mPreundistorter = preundistorter;
            mDefaultParameter = false;
            mUndistortMap = undistortMap;
            computeLevels();
        }

        EIGEN_STRONG_INLINE DistortedVector2d distort(const UndistortedVector2d &input, int lvl) const {
            assertThrow(mDefaultParameter == false, "Default Internal Calibration Parameters");
            return DistortedVector2d(mNewPinholeLevels[lvl].distort(input));
        }

        EIGEN_STRONG_INLINE UndistortedVector2d undistort(const DistortedVector2d &input, int lvl) const {
            assertThrow(mDefaultParameter == false, "Default Internal Calibration Parameters");
            return UndistortedVector2d(mNewPinholeLevels[lvl].undistort(input));
        }

        EIGEN_STRONG_INLINE Matrix33 getK(int lvl) const {
            assertThrow(mDefaultParameter == false, "Default Internal Calibration Parameters");
            return mNewPinholeLevels[lvl].getK();
        }

        EIGEN_STRONG_INLINE PinholeUndistorter getPinhole(int lvl) const {
            assertThrow(mDefaultParameter == false, "Default Internal Calibration Parameters");
            return mNewPinholeLevels[lvl];
        }

        template <typename T> Array2D<T> removeDistortion(const Array2D<T> &input, int outputWidth = 0, int outputHeight = 0) const {

            if (mPreundistorter == nullptr) return input;

            assertThrow(input.getWidth() == mOriginalSize.x() && input.getHeight() == mOriginalSize.y(), "Invalid input size");

            Array2D<T> output(mNewSize.x(), mNewSize.y(), 0.0f);

            int wh = mNewSize.x() * mNewSize.y();

            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int i = 0; i < wh; i++) {
                if (std::isfinite(mUndistortMap.data()[i][0])) {
                    output.data()[i] = input.interpolate(mUndistortMap.data()[i]);
                }
            }

            return output;

        }

        template <typename T> Array2D<T> fastRemoveDistortion(const Array2D<T> &input, int outputWidth = 0, int outputHeight = 0) const {

            if (mPreundistorter == nullptr) return input;

            assertThrow(input.getWidth() == mOriginalSize.x() && input.getHeight() == mOriginalSize.y(), "Invalid input size");

            Array2D<T> output(mNewSize.x(), mNewSize.y(), 0.0f);

            int wh = mNewSize.x() * mNewSize.y();

            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int i = 0; i < wh; i++) {
                if (std::isfinite(mUndistortMap.data()[i][0])) {
                    output.data()[i] = input.get(mUndistortMap.data()[i].x(), mUndistortMap.data()[i].y());
                }
            }

            return output;

        }

        EIGEN_STRONG_INLINE Vector4 getParameters(int lvl) const {
            return mNewPinhole.level(lvl).getParameters();
        }

        EIGEN_STRONG_INLINE PinholeUndistorter getPinhole() const {
            return mNewPinhole;
        }

        EIGEN_STRONG_INLINE void setParameters(Vector4 parameters) {
            mNewPinhole = PinholeUndistorter(parameters);
            computeLevels();
        }

        std::string toString(int level) {
            std::string res = "[InternalCalibration](" + mOriginalPinhole.toString() + " ; ";
            if (mPreundistorter == nullptr) res += "null";
            else res += mPreundistorter->toString();
            res += " ; " + mNewPinhole.level(level).toString() + ")";
            return res;
        }

        Vector2 getOutputSize() const {
            return mNewSize;
        }

    protected:
        void computeUndistortMap();

        EIGEN_STRONG_INLINE void computeLevels() {
            for (int i = 0; i < CML_INTERNALCALIBRATION_LEVELS; i++) {
                mNewPinholeLevels[i] = mNewPinhole.level(i);
            }
        }

    private:
        bool mDefaultParameter = true;
        PinholeUndistorter mOriginalPinhole, mNewPinhole;
        Vector2 mOriginalSize, mNewSize;
        Undistorter *mPreundistorter = nullptr;
        Array2D<Vector2f> mUndistortMap;

        PinholeUndistorter mNewPinholeLevels[CML_INTERNALCALIBRATION_LEVELS];

    };


    InternalCalibration* parseInternalTumCalibration(std::string path, Vector2i outputSize);

    InternalCalibration* parseInternalEurocCalibration(std::string path, Vector2i outputSize);

    InternalCalibration* parseInternalStereopolisCalibration(std::string path, Vector2i outputSize, int top, int bottom);

}


#endif //CML_INTERNALCALIBRATION_H
