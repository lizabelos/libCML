//
// Created by belosth on 13/02/2020.
//

#ifndef CML_ALL_ROTATION_H
#define CML_ALL_ROTATION_H

#include <cml/config.h>

namespace CML {

    template <int N> class Rotation {

    public:
        virtual ~Rotation() { };

        virtual inline Vector3 transform(Vector3 point) const = 0;

        virtual inline Matrix33 matrix() const = 0;

        virtual inline Matrix33 inverseMatrix() const = 0;

        virtual inline Matrix33 matrixDerivative(int i) const = 0;

        virtual inline Matrix33 inverseMatrixDerivative(int i) const = 0;

        virtual inline Matrix<N, 1> getParameters() const = 0;

    };

    class AxisAngle : public Rotation<3> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        const static int N = 3;

        EIGEN_STRONG_INLINE AxisAngle() : mParameters(Vector3::Zero()), mR(Matrix33::Identity()) {

        }

        EIGEN_STRONG_INLINE AxisAngle(const Vector3 &parameters) : mParameters(parameters), mR(hatExp(parameters)) {
            for (int i = 0; i < 3; i++) {
                assertThrow(std::isfinite(parameters(i)), "Invalid parameters");
            }
        }

        EIGEN_STRONG_INLINE AxisAngle(const Matrix33 &m) : mParameters(logHati(m)), mR(m) {
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
                assertThrow(std::isfinite(m(i, j)), "Invalid matrix");
            }
        }

        EIGEN_STRONG_INLINE Matrix33 matrix() const final {
            return mR;
        }

        inline Vector3 transform(Vector3 point) const {
            return matrix() * point;
        }

        EIGEN_STRONG_INLINE AxisAngle inverse() const {
            // Matrix33 m = matrix().transpose();
            // Vector3 p = logHati(m);
            // return AxisAngle(p);
            return AxisAngle(-mParameters, mR.transpose());
        }

        virtual EIGEN_STRONG_INLINE Matrix33 inverseMatrix() const final {
            return mR.transpose();
        }

        virtual EIGEN_STRONG_INLINE Matrix33 matrixDerivative(int i) const final {
            return hatExpDerivative(getParameters(), matrix(), i);
        }

        virtual EIGEN_STRONG_INLINE Matrix33 inverseMatrixDerivative(int i) const final {
            Matrix33 R_prime = hatExpDerivative(getParameters(), matrix(), i);
            return R_prime.transpose();
        }

        EIGEN_STRONG_INLINE AxisAngle operator *(const AxisAngle &other) const {
            Matrix33 m = matrix() * other.matrix();
            Vector3 p = logHati(m);
            return AxisAngle(p);
        }

        EIGEN_STRONG_INLINE Vector3 getParameters() const final {
            return mParameters;
        }

        static Matrix33 hatExp(const Vector3 &d);

        static Vector3 logHati(const Matrix33 &R);

        static Matrix33 hatExpDerivative(Vector3 v, const Matrix33 &R, int i);

        inline AxisAngle fraction(scalar_t f) {
            return AxisAngle((Vector3)(mParameters * f));
        }

    protected:
        EIGEN_STRONG_INLINE AxisAngle(Vector3 parameters, Matrix33 R) : mParameters(parameters), mR(R) {

        }

    private:
        Vector3 mParameters;
        Matrix33 mR;

    };

    class AxisAngleMagnitude : public Rotation<4> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        const static int N = 4;

        AxisAngleMagnitude() : mParameters(Vector4::Zero()), mR(Matrix33::Identity()) {

        }

        AxisAngleMagnitude(scalar_t x, scalar_t y, scalar_t z, scalar_t theta) : mParameters(x, y, z, theta) {
            for (int i = 0; i < 4; i++) {
                assertThrow(std::isfinite(mParameters(i)), "Invalid parameters");
            }
            mR = hatExp(mParameters);
        }

        AxisAngleMagnitude(Vector4 parameters) : mParameters(parameters), mR(hatExp(parameters)) {
            for (int i = 0; i < 4; i++) {
                assertThrow(std::isfinite(parameters(i)), "Invalid parameters");
            }
        }

        AxisAngleMagnitude(Matrix33 m) : mParameters(logHati(m)), mR(m) {
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
                    assertThrow(std::isfinite(m(i, j)), "Invalid matrix");
                }
        }

        inline Vector3 transform(Vector3 point) const {
            return matrix() * point;
        }

        inline Matrix33 matrix() const final {
            return mR;
        }

        inline AxisAngleMagnitude inverse() const {
            // Matrix33 m = matrix().transpose();
            // Vector3 p = logHati(m);
            // return AxisAngle(p);
            return AxisAngleMagnitude(-mParameters, mR.transpose());
        }

        virtual inline Matrix33 inverseMatrix() const final {
            return mR.transpose();
        }

        virtual inline Matrix33 matrixDerivative(int i) const final {
            return hatExpDerivative(getParameters(), matrix(), i);
        }

        virtual inline Matrix33 inverseMatrixDerivative(int i) const final {
            Matrix33 R_prime = hatExpDerivative(getParameters(), matrix(), i);
            return R_prime.transpose();
        }

        inline AxisAngleMagnitude operator *(const AxisAngleMagnitude &other) const {
            Matrix33 m = matrix() * other.matrix();
            Vector4 p = logHati(m);
            return AxisAngleMagnitude(p);
        }

        inline Vector4 getParameters() const final {
            return mParameters;
        }

        static Matrix33 hatExp(const Vector4 &d);

        static Vector4 logHati(const Matrix33 &R);

        static Matrix33 hatExpDerivative(Vector4 v, const Matrix33 &R, int i);

        inline scalar_t x() const {
            return mParameters(0);
        }

        inline scalar_t y() const {
            return mParameters(1);
        }

        inline scalar_t z() const {
            return mParameters(2);
        }

        inline scalar_t theta() const {
            return mParameters(3);
        }

    protected:
        AxisAngleMagnitude(Vector4 parameters, Matrix33 R) : mParameters(parameters), mR(R) {

        }

    private:
        Vector4 mParameters;
        Matrix33 mR;

    };

    class Quaternion  : public Rotation<4> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        const static int N = 4;

        Quaternion() : mParameters(Vector4(1, 0, 0, 0)), mR(Matrix33::Identity()) {
            assertGoodNorm();
        }

        Quaternion(scalar_t w, scalar_t x, scalar_t y, scalar_t z) {
            Vector4 parameters(w, x, y, z);
            for (int i = 0; i < 4; i++) {
                assertThrow(std::isfinite(parameters(i)), "Invalid parameters");
            }
            mParameters = parameters;
            assertGoodNorm();
            mParameters.normalize();
            mR = hatExp(mParameters);
        }

        Quaternion(Vector4 parameters) {
            for (int i = 0; i < 4; i++) {
                assertThrow(std::isfinite(parameters(i)), "Invalid parameters");
            }
            mParameters = parameters;
            assertGoodNorm();
            mParameters.normalize();
            mR = hatExp(mParameters);
        }

        Quaternion(Matrix33 m) : mParameters(logHati(m)), mR(m) {
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
                    assertThrow(std::isfinite(m(i, j)), "Invalid matrix");
            }
            assertGoodNorm();
            mParameters.normalize();
        }

        inline Vector3 transform(Vector3 point) const {
            return Eigen::Quaternion<scalar_t>(w(), x(), y(), z())._transformVector(point);
        }

        inline Matrix33 matrix() const final {
            return mR;
        }

        inline Quaternion inverse() const {
            Matrix33 m = matrix().transpose();
            Vector4 p = logHati(m);
            return Quaternion(p);
        }

        virtual inline Matrix33 inverseMatrix() const final {
            return mR.transpose();
        }

        virtual inline Matrix33 matrixDerivative(int i) const final {
            return hatExpDerivative(getParameters(), matrix(), i);
        }

        virtual inline Matrix33 inverseMatrixDerivative(int i) const final {
            Matrix33 R_prime = hatExpDerivative(getParameters(), matrix(), i);
            return R_prime.transpose();
        }

        inline Quaternion operator *(const Quaternion &b) const {
            const Quaternion &a = *this;
            return Quaternion(
                    a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
                    a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
                    a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
                    a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x());
        }

        inline Vector4 getParameters() const final {
            return mParameters;
        }

        inline scalar_t w() const {
            return mParameters(0);
        }

        inline scalar_t x() const {
            return mParameters(1);
        }

        inline scalar_t y() const {
            return mParameters(2);
        }

        inline scalar_t z() const {
            return mParameters(3);
        }

        static Matrix33 hatExp(const Vector4 &d);

        static Vector4 logHati(const Matrix33 &R);

        static Matrix33 hatExpDerivative(Vector4 v, const Matrix33 &R, int i);

        inline void assertGoodNorm() {
            assertThrow(mParameters.norm() > 0.99 && mParameters.norm() < 1.1, "Invalid quaterion norm : " + std::to_string(mParameters.norm()));
        }

        inline Eigen::Quaternion<scalar_t> toEigen() const {
            return Eigen::Quaternion<scalar_t>(w(), x(), y(), z());
        }

    protected:
        Quaternion(Vector4 parameters, Matrix33 R) : mParameters(parameters), mR(R) {

        }

    private:
        Vector4 mParameters;
        Matrix33 mR;

    };

}

#endif //CML_ALL_ROTATION_H
