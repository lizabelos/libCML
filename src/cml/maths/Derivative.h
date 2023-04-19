//
// Created by belosth on 24/02/2020.
//

#ifndef CML_ALL_DERIVATIVE_H
#define CML_ALL_DERIVATIVE_H

#include <Eigen/Dense>

namespace CML {

    EIGEN_STRONG_INLINE Matrix33 skew(const Vector3 &v) {

        Matrix33 s = Matrix33::Zero();
        s(1, 0) = -v(2);
        s(0, 1) = v(2);

        s(2, 0) = v(1);
        s(0, 2) = -v(1);

        s(2, 1) = -v(0);
        s(1, 2) = v(0);

        return s;

    }

    namespace LossFunction {

        EIGEN_STRONG_INLINE scalar_t huber(scalar_t v, scalar_t th) {
            scalar_t hw = fabs(v) < th ? 1 : th / fabs(v);
            return hw * v * v * (2 - hw);
        }

        EIGEN_STRONG_INLINE scalar_t tukey(scalar_t v, scalar_t th) {
            if (fabs(v) > th) return 0;
            scalar_t l = (scalar_t(1.0) - (v * v) / (th * th));
            return v * l * l;
        }

    }

    namespace Derivative {

        EIGEN_STRONG_INLINE Vector2 hnormalized(const Vector3 &v, const Vector3 &d) {
            return (d.head(2) * v.z() - v.head(2) * d.z()) / (v.z() * v.z());
        }

        template <typename I> scalar_t interpolate(I image, const Vector2 &pos, const Vector2 &d) {
            Vector2 interpolation = image.interpolateGradient(pos);
            return interpolation.x() * d.x() + interpolation.y() * d.y();
        }

        EIGEN_STRONG_INLINE scalar_t interpolate(const Vector2 &interpolation, const Vector2 &d) {
            return interpolation.x() * d.x() + interpolation.y() * d.y();
        }

        EIGEN_STRONG_INLINE scalar_t abs(scalar_t v, scalar_t d) {
            return (fabs(v)/v)*d;
        }

        EIGEN_STRONG_INLINE scalar_t squaredNorm(const Vector2 &v, const Vector2 &d) {
            return scalar_t(2) * d.x() * v.x() + scalar_t(2) * d.y() * v.y();
        }

        EIGEN_STRONG_INLINE scalar_t sqrt(scalar_t v, scalar_t d) {
            if (v == 0) {
                return 0;
            }
            return d / (scalar_t(2) * v);
        }

        EIGEN_STRONG_INLINE Matrix33 hatExp(Vector3 v, const Matrix33 &R, int i) {
            scalar_t theta = v.norm();
            Vector3 ei = Vector3::Zero(); ei(i) = 1;

            if (theta != 0) {
                v = v / theta;
            } else {
                v = ei;
            }

            scalar_t c = cos(theta);
            scalar_t s = sin(theta);
            Matrix33 ssm = skew(v);

            Matrix33 A = c * v(i) * ssm;
            Matrix33 B = s * v(i) * (ssm * ssm);
            Matrix33 C = Matrix33::Zero();
            Matrix33 D = Matrix33::Zero();
            if (theta != 0) {
                C = (s / theta) * skew(ei - v(i) * v);
                D = ((scalar_t(1.0) - c) / theta) * (ei * v.transpose() + v * ei.transpose() - 2 * v(i) * (v * v.transpose()));
            }

            return A + B + C + D;

        }

        EIGEN_STRONG_INLINE Matrix33 inverseHatExp(Vector3 v, const Matrix33 &R, int i) {
            Matrix33 R_prime = hatExp(v, R, i);
            return R_prime.transpose();
            // Matrix33 R_inv_prime = -R.transpose() * R_prime * R.transpose();
            // return R_inv_prime * R_prime;
            // inverse(hatExp(x)) = inverse'(hatExp(x))hatExp'(x)

        }

        EIGEN_STRONG_INLINE scalar_t tukey(scalar_t v, scalar_t d, scalar_t th) {
            if (fabs(v) > th) return 0;
            scalar_t  v2 = v * v;
            scalar_t _1_v2 = scalar_t(1) - v2;
            return d * (-scalar_t(4.0) * v2 * _1_v2 + _1_v2 * _1_v2);
        }

    }

}

#endif //CML_ALL_DERIVATIVE_H
