#include "cml/maths/Rotation.h"
#include "cml/maths/Utils.h"

EIGEN_STRONG_INLINE CML::Matrix33 skew(CML::Vector3 v) {

    CML::Matrix33 s = CML::Matrix33::Zero();
    s(1, 0) = -v(2);
    s(0, 1) = v(2);

    s(2, 0) = v(1);
    s(0, 2) = -v(1);

    s(2, 1) = -v(0);
    s(1, 2) = v(0);

    return s;

}

CML::Matrix33 CML::AxisAngle::hatExp(const Vector3 &d) {

    scalar_t theta = d.norm();
    scalar_t x = d(0), y = d(1), z = d(2);

    if (theta != 0) {
        x = d(0) / theta;
        y = d(1) / theta;
        z = d(2) / theta;
    }

    scalar_t c = cos(theta / 2.0), s = sin(theta / 2.0);

    scalar_t x2 = x * x, y2 = y * y, z2 = z * z;
    scalar_t s2 = s * s;

    scalar_t _2xys2 = 2.0 * x * y * s2;
    scalar_t _2xzs2 = 2.0 * x * z * s2;
    scalar_t _2yzs2 = 2.0 * y * z * s2;

    scalar_t _2xcs = 2.0 * x * c * s;
    scalar_t _2ycs = 2.0 * y * c * s;
    scalar_t _2zcs = 2.0 * z * c * s;

    Matrix33 R;
    R(0, 0) = 2.0 * (x2 - 1.0) * s2 + 1.0;
    R(1, 0) = _2xys2 - _2zcs;
    R(2, 0) = _2xzs2 + _2ycs;
    R(0, 1) = _2xys2 + _2zcs;
    R(1, 1) = 2.0 * (y2 - 1.0) * s2 + 1.0;
    R(2, 1) = _2yzs2 - _2xcs;
    R(0, 2) = _2xzs2 - _2ycs;
    R(1, 2) = _2yzs2 + _2xcs;
    R(2, 2) = 2.0 * (z2 - 1.0) * s2 + 1.0;

    return R;
}

CML::Vector3 CML::AxisAngle::logHati(const Matrix33 &R) {

    scalar_t theta = acos((R.trace() - 1.0) / 2.0);

    if (theta == 0 || !std::isfinite(theta)) {
        return Vector3::Zero();
    }

    scalar_t sintheta = sin(theta);

    Matrix33 ssm = theta / (2.0 * sintheta) * (R - R.transpose());

    Vector3 result = Vector3(ssm(1, 2), ssm(2, 0), ssm(0, 1));

    for (int i = 0; i < 3; i++) {
        assertThrow(std::isfinite(result(i)), "Invalid parameters. sin(theta) = " + std::to_string(sintheta) + "; theta = " + std::to_string(theta));
    }

    return result;
}

CML::Matrix33 CML::AxisAngle::hatExpDerivative(Vector3 v, const Matrix33 &R, int i) {

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

CML::Matrix33 CML::AxisAngleMagnitude::hatExp(const Vector4 &d) {

    scalar_t theta = d(3);
    scalar_t x = d(0), y = d(1), z = d(2);

    scalar_t c = cos(theta / 2.0), s = sin(theta / 2.0);

    scalar_t x2 = x * x, y2 = y * y, z2 = z * z;
    scalar_t s2 = s * s;

    scalar_t _2xys2 = 2.0 * x * y * s2;
    scalar_t _2xzs2 = 2.0 * x * z * s2;
    scalar_t _2yzs2 = 2.0 * y * z * s2;

    scalar_t _2xcs = 2.0 * x * c * s;
    scalar_t _2ycs = 2.0 * y * c * s;
    scalar_t _2zcs = 2.0 * z * c * s;

    Matrix33 R;
    R(0, 0) = 2.0 * (x2 - 1.0) * s2 + 1.0;
    R(1, 0) = _2xys2 - _2zcs;
    R(2, 0) = _2xzs2 + _2ycs;
    R(0, 1) = _2xys2 + _2zcs;
    R(1, 1) = 2.0 * (y2 - 1.0) * s2 + 1.0;
    R(2, 1) = _2yzs2 - _2xcs;
    R(0, 2) = _2xzs2 - _2ycs;
    R(1, 2) = _2yzs2 + _2xcs;
    R(2, 2) = 2.0 * (z2 - 1.0) * s2 + 1.0;

    return R;
}

CML::Vector4 CML::AxisAngleMagnitude::logHati(const Matrix33 &R) {

    scalar_t theta = acos((R.trace() - 1.0) / 2.0);

    if (theta == 0 || !std::isfinite(theta)) {
        return Vector4::Zero();
    }

    scalar_t sintheta = sin(theta);

    Matrix33 ssm = theta / (2.0 * sintheta) * (R - R.transpose());

    Vector4 result = Vector4(ssm(1, 2), ssm(2, 0), ssm(0, 1), theta);

    for (int i = 0; i < 3; i++) {
        assertThrow(std::isfinite(result(i)), "Invalid parameters. sin(theta) = " + std::to_string(sintheta) + "; theta = " + std::to_string(theta));
    }

    return result;
}

CML::Matrix33 CML::AxisAngleMagnitude::hatExpDerivative(Vector4 v4, const Matrix33 &R, int i) {

    assertThrow(false, "Not implemented");
    abort();

}

CML::Matrix33 CML::Quaternion::hatExp(const Vector4 &q) {

    scalar_t a = q(0);
    scalar_t b = q(1);
    scalar_t c = q(2);
    scalar_t d = q(3);

    scalar_t _2b2 = scalar_t(2) * b * b;
    scalar_t _2c2 = scalar_t(2) * c * c;
    scalar_t _2d2 = scalar_t(2) * d * d;

    scalar_t _2bc = scalar_t(2) * b * c;
    scalar_t _2ad = scalar_t(2) * a * d;
    scalar_t _2bd = scalar_t(2) * b * d;
    scalar_t _2ac = scalar_t(2) * a * c;
    scalar_t _2cd = scalar_t(2) * c * d;
    scalar_t _2ab = scalar_t(2) * a * b;

    Matrix33 R;

    // Line 0
    R(0, 0) = scalar_t(1) - _2c2 - _2d2;
    R(0, 1) = _2bc - _2ad;
    R(0, 2) = _2bd + _2ac;

    // Line 1
    R(1, 0) = _2bc + _2ad;
    R(1, 1) = scalar_t(1) - _2b2 - _2d2;
    R(1, 2) = _2cd - _2ab;

    // Line 2
    R(2, 0) = _2bd - _2ac;
    R(2, 1) = _2cd + _2ab;
    R(2, 2) = scalar_t(1) - _2b2 - _2c2;

    return R;
}

CML::Vector4 CML::Quaternion::logHati(const Matrix33 &R) {

    Vector4 q;

    const int w = 0, x = 1, y = 2, z = 3;

    q(w) = sqrt(max(scalar_t(0), scalar_t(1) + R(0, 0) + R(1, 1) + R(2, 2))) / scalar_t(2);
    q(x) = sqrt(max(scalar_t(0), scalar_t(1) + R(0, 0) - R(1, 1) - R(2, 2))) / scalar_t(2);
    q(y) = sqrt(max(scalar_t(0), scalar_t(1) - R(0, 0) + R(1, 1) - R(2, 2))) / scalar_t(2);
    q(z) = sqrt(max(scalar_t(0), scalar_t(1) - R(0, 0) - R(1, 1) + R(2, 2))) / scalar_t(2);
    q(x) = std::copysign(q(x), R(2, 1) - R(1, 2));
    q(y) = std::copysign(q(y), R(0, 2) - R(2, 0));
    q(z) = std::copysign(q(z), R(1, 0) - R(0, 1));

    return q;

}

CML::Matrix33 CML::Quaternion::hatExpDerivative(Vector4 q, const Matrix33 &R, int i) {

    scalar_t a = q(0);
    scalar_t b = q(1);
    scalar_t c = q(2);
    scalar_t d = q(3);

    scalar_t _2b2 = scalar_t(0);
    scalar_t _2c2 = scalar_t(0);
    scalar_t _2d2 = scalar_t(0);

    scalar_t _2bc = scalar_t(0);
    scalar_t _2ad = scalar_t(0);
    scalar_t _2bd = scalar_t(0);
    scalar_t _2ac = scalar_t(0);
    scalar_t _2cd = scalar_t(0);
    scalar_t _2ab = scalar_t(0);

    switch (i) {
        case 0: // a
            _2ad = scalar_t(2) * d;
            _2ac = scalar_t(2) * c;
            _2ab = scalar_t(2) * b;
            break;
        case 1: // b
            _2b2 = scalar_t(4) * b;

            _2bc = scalar_t(2) * c;
            _2bd = scalar_t(2) * d;
            _2ab = scalar_t(2) * a;
            break;
        case 2: // c
            _2c2 = scalar_t(4) * c;

            _2bc = scalar_t(2) * b;
            _2ac = scalar_t(2) * a;
            _2cd = scalar_t(2) * d;
            break;
        case 3: // d
            _2d2 = scalar_t(4) * d;

            _2ad = scalar_t(2) * a;
            _2bd = scalar_t(2) * b;
            _2cd = scalar_t(2) * c;
            break;
    }

    Matrix33 Rd;

    // Line 0
    Rd(0, 0) = - _2c2 - _2d2;
    Rd(0, 1) = _2bc - _2ad;
    Rd(0, 2) = _2bd + _2ac;

    // Line 1
    Rd(1, 0) = _2bc + _2ad;
    Rd(1, 1) = - _2b2 - _2d2;
    Rd(1, 2) = _2cd - _2ab;

    // Line 2
    Rd(2, 0) = _2bd - _2ac;
    Rd(2, 1) = _2cd + _2ab;
    Rd(2, 2) = - _2b2 - _2c2;

    return Rd;

}