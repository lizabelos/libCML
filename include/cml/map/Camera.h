//
// Created by tbelos on 24/04/19.
//

#ifndef CML_CAMERA_H
#define CML_CAMERA_H

#include <memory>
#include <atomic>
#include <random>

#include "cml/config.h"
#include "cml/maths/Rotation.h"
#include "cml/maths/Derivative.h"

namespace CML {

    EIGEN_STRONG_INLINE void assertHaveGoodCameraTranslation(const Vector3 &translation) {
        for (int i = 0; i < 3; i++) {
            assertThrow(std::isfinite(translation(i)), "Invalid translation");
        }
    }

    /**
     * @brief Camera represented with a SE3 transformation
     */
    class Camera {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Default constructor for a camera, initialized at origin and pointing to the z-axis
         */
        EIGEN_STRONG_INLINE Camera() {
            mTranslation = Vector3(0,0,0);
            mRotation = Matrix33::Identity();
        }

        /**
         * @brief Construct a camera with a translation vector and a quaternion
         * @param translation The translation of the SE3 transformation
         * @param rotation The quaternion of the SE3 transformation
         */
        EIGEN_STRONG_INLINE Camera(const Vector3 &translation, const Quaternion &rotation) {
            mTranslation = translation;
            mRotation = rotation.matrix();
            mQuaternion = rotation;
            assertHaveGoodCameraTranslation(mTranslation);
            mQuaternionIsComputed = true;
            mAxisAngleIsComputed = false;
            //assertThrow(mRotation.getParameters().isFinite(), "Camera constructor : rotation must be finite");
        }

        /**
         * @brief Construct a camera with a translation vector and a axis-angle rotation
         * @param translation The translation of the SE3 transformation
         * @param rotation The axis-angle of the SE3 transformation
         */
        EIGEN_STRONG_INLINE Camera(const Vector3 &translation, const AxisAngle &rotation) {
            mTranslation = translation;
            mRotation = rotation.matrix();
            mAxisAngle = rotation;
            assertHaveGoodCameraTranslation(mTranslation);
            mQuaternionIsComputed = false;
            mAxisAngleIsComputed = true;
            //assertThrow(mRotation.getParameters().isFinite(), "Camera constructor : rotation must be finite");
        }

        /**
         * @brief Construct a camera with a translation vector and a matrix 3x3 rotation
         * @param translation The translation of the SE3 transformation
         * @param rotation The 3x3 rotation matrix of the SE3 transformation
         */
        EIGEN_STRONG_INLINE Camera(const Vector3 &translation, const Matrix33 &rotation) {
            mTranslation = translation;
            mRotation = rotation;
            assertHaveGoodCameraTranslation(mTranslation);
            mQuaternionIsComputed = false;
            mAxisAngleIsComputed = false;
            //assertThrow(mRotation.getParameters().isFinite(), "Camera constructor : rotation must be finite");
        }

        /**
         * @brief Construct a camera with a translation vector and a axis-angle-magnitude rotation
         * @param translation The translation of the SE3 transformation
         * @param rotation The axis-angle-magnitude of the SE3 transformation
         */
        EIGEN_STRONG_INLINE Camera(const Vector3 &translation, const AxisAngleMagnitude &rotation) {
            mTranslation = translation;
            mRotation = rotation.matrix();
            assertHaveGoodCameraTranslation(mTranslation);
            mQuaternionIsComputed = false;
            mAxisAngleIsComputed = false;
            //assertThrow(mRotation.getParameters().isFinite(), "Camera constructor : rotation must be finite");
        }

        /**
         * @brief Construct a camera from a origin and a "look-at" vector
         * @param center Origin of the camera
         * @param eye Where the camera look at
         * @return The new camera
         */
        static Camera fromDirectionVector(const Vector3 &center, const Vector3 &eye) {
            Vector3 direction = (eye - center).normalized();
            Vector3 forward(0, 0, 1);

            //scalar_t theta = acos(forward.dot(direction));
            //Vector3 axis = forward.cross(direction).normalized();

            //AxisAngleMagnitude R = AxisAngleMagnitude(axis.x(), axis.y(), axis.z(), theta);
            Eigen::Quaternion<scalar_t> q = Eigen::Quaternion<scalar_t>::FromTwoVectors(direction, forward);
            Vector3 t = -(q.matrix() * center);

            return {t, q.matrix()};
        }

        /**
         * @brief Construct a camera from a normalized matrix
         * @param M A normalized matrix
         * @return The new camera
         */
        static Camera fromNormalizedMatrix(const Matrix44 &M) {
            return Camera(M.col(3).head<3>(), M.block<3, 3>(0, 0));
        }

        /**
         * @brief Construct a camera from a normalized matrix
         * @param M A normalized matrix
         * @return The new camera
         */
        static Camera fromNormalizedMatrix34(const Matrix34 &M) {
            return Camera(M.col(3).head<3>(), M.block<3, 3>(0, 0));
        }

        /**
         * @brief Construct a camera from a nullspace matrix
         * @param M A nullspace matrix
         * @return The new camera
         */
        static Camera fromNullspaceMatrix(const Matrix44 &M) {
            Matrix34 E = Matrix34::Identity();
            E.block(0, 3, 3, 1) = -M.col(3).head<3>();
            Matrix33 R = M.block<3, 3>(0, 0).transpose();
            return fromNormalizedMatrix34(R * E);
        }

        /**
         * @brief Construct a camera at origin, looking at the z-axis
         * @return The new camera
         */
        EIGEN_STRONG_INLINE static Camera identity() {
            return Camera();
        }

        /**
         * @brief Construct a random camera
         * @return The new randomly generated camera
         */
        EIGEN_STRONG_INLINE static Camera random() {
            std::random_device rd;
            std::mt19937 e2(rd());
            std::uniform_real_distribution<> dist(-10, 10);

            Vector3 t;
            Vector<Quaternion::N> a;
            for (int i = 0; i < 3; i++) t(i) = dist(e2);
            for (int i = 0; i < Quaternion::N; i++) a(i) = dist(e2);

            return Camera(t, Quaternion(a));
        }

        /**
         * @return Return the translation of the camera
         */
        EIGEN_STRONG_INLINE Vector3 getTranslation() const {
            return mTranslation;
        }

        /**
         * @return Return the rotation of the camera in the form of a quaternion
         */
        EIGEN_STRONG_INLINE Quaternion getQuaternion() const {
            if (!mQuaternionIsComputed) {
                mQuaternion = Quaternion(mRotation);
                mQuaternionIsComputed = true;
            }
            return mQuaternion;
        }

        /**
         * @return Return the rotation of the camera in the form of an axis-angle
         */
        EIGEN_STRONG_INLINE AxisAngle getAxisAngle() const {
            if (!mAxisAngleIsComputed) {
                mAxisAngle = AxisAngle(mRotation);
                mAxisAngleIsComputed = true;
            }
            return AxisAngle(mRotation);
        }

        /**
         * @return Return the rotation of the camera in the form of a 3x3 rotation matrix
         */
        EIGEN_STRONG_INLINE Matrix33 getRotationMatrix() const {
            return mRotation;
        }

        /**
         * @param translation The new translation of the camera
         * @return Return a new camera with the same rotation, but with the given translation
         */
        EIGEN_STRONG_INLINE Camera withTranslation(const Vector3 &translation) const {
            assertHaveGoodCameraTranslation(translation);
            Camera copy = *this;
            copy.mTranslation = translation;
            return copy;
        }

        /**
         * @param translation The new translation of the camera
         * @return Return a new camera with the same translation, but with the given rotation
         */
        EIGEN_STRONG_INLINE Camera withQuaternion(const Quaternion &rotation) const {
            Camera copy = *this;
            copy.mQuaternion = rotation;
            copy.mRotation = rotation.matrix();
            copy.mQuaternionIsComputed = true;
            copy.mAxisAngleIsComputed = false;
            return copy;
        }

        /**
         * @param translation The new translation of the camera
         * @return Return a new camera with the same translation, but with the given rotation
         */
        EIGEN_STRONG_INLINE Camera withAxisAngle(const AxisAngle &axisAngle) const {
            Camera copy = *this;
            copy.mAxisAngle = axisAngle;
            copy.mRotation = axisAngle.matrix();
            copy.mAxisAngleIsComputed = true;
            copy.mQuaternionIsComputed = false;
            return copy;
        }

        /**
         * @param translation The new translation of the camera
         * @return Return a new camera with the same translation, but with the given rotation
         */
        EIGEN_STRONG_INLINE Camera withRotationMatrix(const Matrix33 &rotationMatrix) const {
            Camera copy = *this;
            copy.mRotation = rotationMatrix;
            copy.mAxisAngleIsComputed = false;
            copy.mQuaternionIsComputed = false;
            return copy;
        }

        /**
         * @return Return a new camera with the inverted SE3 transformation
         */
        EIGEN_STRONG_INLINE Camera inverse() const {
            Matrix33 inverseRotationMatrix = getRotationMatrix().transpose();
            return {inverseRotationMatrix * -getTranslation(), inverseRotationMatrix};
        }

        /**
         * @return Return the adjoint matrix of the SE3 transformation
         */
        EIGEN_STRONG_INLINE Matrix<6, 6> adjoint() const {
            Matrix33 rotationMatrix = mRotation.matrix();
            Matrix33 translationHat;
            translationHat <<    0    , -mTranslation.z(), mTranslation.y(),
                    mTranslation.z() , 0    , -mTranslation.x(),
                    -mTranslation.y(), mTranslation.x() ,     0;

            Matrix<6, 6> res;
            res.block(0,0,3,3) = rotationMatrix;
            res.block(3,3,3,3) = rotationMatrix;
            res.block(0,3,3,3) = translationHat * rotationMatrix;
            res.block(3,0,3,3) = Matrix<3,3>::Zero(3,3);
            return res;
        }

        /**
         * @brief Compose two camera
         * @param other The camera to compose with
         * @return The composition of this camera with other camera
         */
        EIGEN_STRONG_INLINE Camera compose(const Camera &other) const {
            return {other.transform(getTranslation()), other.getQuaternion() * getQuaternion()};
            //return {transform(other.getTranslation()), getQuaternion() * other.getQuaternion()};
        }

        /**
         * @return Return a camera that, when composed to this camera, return the other camera
         */
        EIGEN_STRONG_INLINE Camera to(const Camera &other) const {
            return inverse().compose(other);
            // return {untransform(other.getTranslation()), mRotation.inverse() * other.mRotation};
        }

        /**
         * @brief Apply the SE3 transformation to a vector
         * @param P The vector on which to apply the transformation
         * @return The transformed vector
         */
        EIGEN_STRONG_INLINE Vector3 transform(const Vector3 &P) const {
            if (mQuaternionIsComputed) {
                return mQuaternion.transform(P) + mTranslation;
            }
            if (mAxisAngleIsComputed) {
                return mAxisAngle.transform(P) + mTranslation;
            }
            return getRotationMatrix() * P + mTranslation;
        }

        EIGEN_STRONG_INLINE Vector3 derivativeOfTransformWrtTranslation(int i) const {
            Vector3 translationWrtTranslationI = Vector3::Zero();
            translationWrtTranslationI(i) = 1;
            return mRotation.matrix() * translationWrtTranslationI;
        }

        EIGEN_STRONG_INLINE Vector3 derivativeOfTransformWrtRotation(const Vector3 &P, int i) const {
            return Quaternion(mRotation).matrixDerivative(i) * (P + mTranslation);
        }

        EIGEN_STRONG_INLINE Vector3 derivativeOfTransformWrtPoint(int i) const {
            Vector3 pointWrtPointI = Vector3::Zero();
            pointWrtPointI(i) = 1;
            return mRotation.matrix() * pointWrtPointI;
        }

        /**
         * @brief Apply the inverse SE3 transformation to a vector
         * @param P The vector on which to apply the inverse transformation
         * @return The un-transformed vector
         */
        EIGEN_STRONG_INLINE Vector3 untransform(const Vector3 &P) const {
            return mRotation.matrix().transpose() * (P - mTranslation);
        }

        EIGEN_STRONG_INLINE Vector3 derivativeOfUntransformWrtTranslation(int i) const {
            Vector3 minusTranslationWrtTranslationI = Vector3::Zero();
            minusTranslationWrtTranslationI(i) = -1;
            return mRotation.matrix().transpose() * minusTranslationWrtTranslationI;
        }

        EIGEN_STRONG_INLINE Vector3 derivativeOfUntransformWrtRotation(const Vector3 &P, int i) const {
            return Quaternion(mRotation).inverseMatrixDerivative(i) * (P - mTranslation);
        }

        /**
         * @brief Project a 3D point into the camera
         * @param P The point to project
         * @return The projected point
         */
        EIGEN_STRONG_INLINE UndistortedVector2d project(const Vector3 &P) const {
            return UndistortedVector2d(transform(P).hnormalized());
        }

        /**
         * @brief Project a 3D point into the camera, but assuming the point is a distance depth of the camera
         * @param P The point to project
         * @param depth The depth of the point
         * @return The projected point
         */
        EIGEN_STRONG_INLINE UndistortedVector2d project(const Vector3 &P, scalar_t &depth) const {
            Vector3 transformed = transform(P);
            depth = transformed.z();
            return UndistortedVector2d(transformed.hnormalized());
        }

        EIGEN_STRONG_INLINE Vector2 derivativeOfProjectWrtTranslation(const Vector3 &v, int i) const {
            Vector3 d = derivativeOfTransformWrtTranslation(i);
            return Derivative::hnormalized(v, d);
        }

        EIGEN_STRONG_INLINE Vector2 derivativeOfProjectWrtRotation(Vector3 P, Vector3 v, int i) const {
            Vector3 d = derivativeOfTransformWrtRotation(P, i);
            return Derivative::hnormalized(v, d);
        }

        EIGEN_STRONG_INLINE Vector2 derivativeOfProjectWrtPoint(Vector3 v, int i) const {
            Vector3 d = derivativeOfTransformWrtPoint(i);
            return Derivative::hnormalized(v, d);
        }

        /**
         * @brief Compute the parallax of a 3D point according to two camera
         * @param other The other camera
         * @param P The point to compute the parallax
         * @return The parallax of the point
         */
        EIGEN_STRONG_INLINE scalar_t parallax(const Camera &other, const Vector3 &P) const {

            Vector3 A = mTranslation - P;
            Vector3 B = other.mTranslation - P;

            A = A / A.norm();
            B = B / B.norm();

            return A.dot(B);

        }

        /**
         * @brief Compute the parallax of two 2d points (representing the same 3d points) according to two camera
         * @param c2 The other camera
         * @param x1 The first 2d point to compute the parallax
         * @param x2 The second 2d point to compute the parallax
         * @return The parallax of the 3d point
         */
        EIGEN_STRONG_INLINE scalar_t parallax(const Camera &c2, const UndistortedVector2d &x1, const UndistortedVector2d &x2) const {
            const Camera &c1 = *this;
            Vector3 ray1 = c1.getRotationMatrix().transpose() * x1.homogeneous();
            Vector3 ray2 = c2.getRotationMatrix().transpose() * x2.homogeneous();
            return ray1.dot(ray2)/(ray1.norm() * ray2.norm());
        }

        /**
         * @return Return the essential matrix of this camera
         */
        EIGEN_STRONG_INLINE Matrix33 essential() const {

            Matrix33 S = mRotation.matrix();
            Matrix33 T;
            T <<    0    , -mTranslation.z(), mTranslation.y(),
                    mTranslation.z() , 0    , -mTranslation.x(),
                    -mTranslation.y(), mTranslation.x() ,     0;
            return T; // TODO

            Matrix33 E = S * T;

            return E;

        }

        /**
         * @return The normalized camera matrix with a 4x4 size
         */
        EIGEN_STRONG_INLINE Matrix44 normalizedCameraMatrix44() const {
            Matrix44 E = Matrix44::Identity();
            E.block(0, 3, 3, 1) = getTranslation();
            E.block(0, 0, 3, 3) = getRotationMatrix();
            return E;
        }

        /**
         * @return The normalized camera matrix with a 3x4 size
         */
        EIGEN_STRONG_INLINE Matrix34 normalizedCameraMatrixMatrix34() const {
            Matrix34 E = Matrix34::Identity();
            E.block(0, 3, 3, 1) = getTranslation();
            E.block(0, 0, 3, 3) = getRotationMatrix();
            return E;
        }

        /**
         * @return The nullspace camera matrix with a 4x4 size
         */
        EIGEN_STRONG_INLINE Matrix44 nullspaceCameraMatrix44() const {
            Matrix44 E = Matrix44::Identity();

            Matrix33 Rt = getRotationMatrix().transpose();
            Vector3 n = -Rt * getTranslation();

            E.block(0, 3, 3, 1) = n;
            E.block(0, 0, 3, 3) = Rt;

            return E;
        }

        /**
         * @return The nullspace camera matrix with a 3x4 size
         */
        EIGEN_STRONG_INLINE Matrix34 nullspaceCameraMatrix34() const {
            Matrix34 E = Matrix34::Identity();

            Matrix33 Rt = getRotationMatrix().transpose();
            Vector3 n = -Rt * getTranslation();

            E.block(0, 3, 3, 1) = n;
            E.block(0, 0, 3, 3) = Rt;

            return E;
        }

        /**
         * @return Flipped the camera arround the x axis
         */
        EIGEN_STRONG_INLINE Camera flipX() const {
            Quaternion q(-getQuaternion().w(), -getQuaternion().x(), getQuaternion().y(), getQuaternion().z());
            Vector3 t(-getTranslation().x(), getTranslation().y(), getTranslation().z());
            return {t, q};
        }

        /**
         * @return Flipped the camera arround the y axis
         */
        EIGEN_STRONG_INLINE Camera flipY() const {
            Quaternion q(-getQuaternion().w(), getQuaternion().x(), -getQuaternion().y(), getQuaternion().z());
            Vector3 t(getTranslation().x(), -getTranslation().y(), getTranslation().z());
            return {t, q};
        }

        /**
         * @return Flipped the camera arround the z axis
         */
        EIGEN_STRONG_INLINE Camera flipZ() const {
            Quaternion q(-getQuaternion().w(), getQuaternion().x(), getQuaternion().y(), -getQuaternion().z());
            Vector3 t(getTranslation().x(), getTranslation().y(), -getTranslation().z());
            return {t, q};
        }

        /**
         * @return A new camera with the translation multiplied by v
         */
        EIGEN_STRONG_INLINE Camera scale(scalar_t v) const {
            Quaternion q = getQuaternion();
            Vector3 t = getTranslation() * v;
            return {t, q};
        }

        /**
         * @return Return the eye of the camera
         */
        EIGEN_STRONG_INLINE Vector3 eye() const {
            return untransform(Vector3(0, 0, 0));
        }

        /**
         * @return Return the look-at of the camera (at a distance of 1 of the camera)
         */
        EIGEN_STRONG_INLINE Vector3 look() const {
            return untransform(Vector3(0, 0, 1));
        }

        /**
         * @return Return the up vector of the camera (at a distance of 1 of the camera)
         */
        EIGEN_STRONG_INLINE Vector3 up() const {
            return untransform(Vector3(0, -1, 0));
        }

        /**
         * @return Return the true if a point is in front of the camera
         */
        EIGEN_STRONG_INLINE bool inFront(const Vector3 &point) const {
            return transform(point).z() > 0;
        }

        /**
         * @return Return a camera with a fraction of the SE3 transformation of this camera
         */
        EIGEN_STRONG_INLINE Camera fraction(scalar_t f) const {
            return Camera(getTranslation() * f, getAxisAngle().fraction(f));
        }

    private:
        Vector3 mTranslation;
        Matrix33 mRotation;

        mutable bool mQuaternionIsComputed = false;
        mutable Quaternion mQuaternion;

        mutable bool mAxisAngleIsComputed = false;
        mutable AxisAngle mAxisAngle;

    };

    /**
     * @return The true if two camera are almost equal
     */
    EIGEN_STRONG_INLINE bool almostEqual(const Camera &a, const Camera &b) {
        return almostEqual(a.getTranslation(), b.getTranslation()) && almostEqual(a.getRotationMatrix(), b.getRotationMatrix());
    }

    /**
     * @brief A class that encapsulate a 3D point to securely manipulate it
     */
    class WorldPoint {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief Default constructor for a 3d point, initialized at origin
         */
        EIGEN_STRONG_INLINE WorldPoint() {
            mAbsolute = Vector3(0, 0, 0);
        }

        /**
         * @brief Construct a 3d point from it's absolute position
         * @return Return the constructed 3d point
         */
        EIGEN_STRONG_INLINE static WorldPoint fromAbsolute(const Vector3 &absolute) {
            return WorldPoint(absolute);
        }

        /**
         * @brief Construct a 3d point from it's relative position
         * @param relative The relative position of the 3d point
         * @param reference The reference of the 3d point
         * @return Return the constructed 3d point
         */
        EIGEN_STRONG_INLINE static WorldPoint fromRelativeCoordinates(const Vector3 &relative, const Camera &reference) {
            return WorldPoint(reference.untransform(relative));
        }

        /**
         * @brief Construct a 3d point from it's 2d position and it's depth
         * @param idepth The relative depth of the point according to the reference
         * @param point The 2d position of the point inside the reference
         * @param reference The reference of the 2d point
         * @return Return the constructed 3d point
         */
        EIGEN_STRONG_INLINE static WorldPoint fromInverseDepth(scalar_t idepth, const UndistortedVector2d &point, const Camera &reference) {
            return WorldPoint::fromRelativeCoordinates(point.homogeneous() / idepth, reference);
        }

        /**
         * @return Return the absolute 3d position of the point
         */
        EIGEN_STRONG_INLINE Vector3 absolute() const {
            return mAbsolute;
        }

        /**
         * @param reference The reference
         * @return Return the relative to the reference 3d position of the point
         */
        EIGEN_STRONG_INLINE Vector3 relative(const Camera &reference) const {
            return reference.transform(mAbsolute);
        }

        /**
         * @brief Project this point to a camera
         * @param reference The camera to project the point
         * @return The projected 2d point
         */
        EIGEN_STRONG_INLINE UndistortedVector2d project(const Camera &reference) const {
            return reference.project(mAbsolute);
        }

        /**
         * @brief Compute the distance between a camera and a point
         * @return The distance between the camera and this point
         */
        EIGEN_STRONG_INLINE scalar_t distance(const Camera &reference) const {
            return (mAbsolute - reference.getTranslation()).norm();
        }

    protected:
        EIGEN_STRONG_INLINE WorldPoint(Vector3 absolute) {
            assertThrow(absolute.allFinite(), "Trying to create non finite world point !");
            mAbsolute = absolute;
        }

    private:
        Vector3 mAbsolute;

    };

    EIGEN_STRONG_INLINE std::ostream& operator<<(std::ostream& os, const Camera &camera) {
        os << "[Camera](" << camera.getTranslation().transpose() << " ; " << camera.getAxisAngle().getParameters().transpose() << ")";
        return os;
    }

    EIGEN_STRONG_INLINE std::ostream& operator<<(std::ostream& os, const WorldPoint &camera) {
        os << "[WorldPoint](" << camera.absolute().transpose() << ")";
        return os;
    }

    /**
     * @brief You can use the operator > to call the "to" method between two camera
     */
    EIGEN_STRONG_INLINE Camera operator>(const Camera &a, const Camera &b) {
        return a.to(b);
    }

    /**
     * @brief You can use the operator * to call the "compose" method between two camera
     */
    EIGEN_STRONG_INLINE Camera operator*(const Camera &a, const Camera &b) {
        return a.compose(b);
    }

    /**
     * @brief You can use the operator * to call the "fraction" method between two camera
     */
    EIGEN_STRONG_INLINE Camera operator*(const Camera &a, scalar_t b) {
        return a.fraction(b);
    }

    /**
     * @brief Compute the velocity between two cameras
     * @return The velocity between the two cameras
     */
    inline Camera computeVelocity(const Camera &last, const Camera &prelast, int num = 1) {
        Camera lastMotion = prelast.to(last);

        if (num > 1) {
            lastMotion = lastMotion * (1.0 / (scalar_t)num);
        }

        return lastMotion;
    }

    /**
     * @brief Compute a constant velocity motion model from two camera
     * @return A list of camera with a generated constant velocity motion model
     */
    inline List<Camera> constantVelocityMotionModel(const Camera &last, const Camera &prelast, int num = 1) {

        List<Camera> motionToTries;

        Camera lastMotion = prelast.to(last);

        if (num > 1) {
            lastMotion = lastMotion * (1.0 / (scalar_t)num);
        }

        // get last delta-movement.
        motionToTries.emplace_back(lastMotion);
        motionToTries.emplace_back(lastMotion * lastMotion);
        motionToTries.emplace_back(lastMotion * 0.5);
        motionToTries.emplace_back(Camera());

        // just try a TON of different initializations (all rotations). In the end,
        // if they don't work they will only be tried on the coarsest level, which is super fast anyway.
        // also, if tracking rails here we loose, so we really, really want to avoid that.
        for(scalar_t rotDelta = 0.02; rotDelta < 0.05; rotDelta = rotDelta + 0.01)
        {
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, 0, 0)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, rotDelta, 0)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, 0, rotDelta)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, 0, 0)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, -rotDelta, 0)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, 0, -rotDelta)));			// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, rotDelta, 0)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, rotDelta, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, 0, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, rotDelta, 0)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, -rotDelta, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, 0, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, -rotDelta, 0)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, 0, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, -rotDelta, 0)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, 0, -rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, 0, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, -rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, -rotDelta, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, -rotDelta, rotDelta, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, -rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, -rotDelta, rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, rotDelta, -rotDelta)));	// assume constant motion.
            motionToTries.emplace_back(Camera(Vector3(0, 0, 0), Quaternion(1, rotDelta, rotDelta, rotDelta)));	// assume constant motion.
        }


        return motionToTries;

    }

    // From ORB SLAM 2
    inline bool checkDistEpipolarLine(const CML::Corner &kp1, const CML::Corner &kp2, const CML::Matrix33 &F12, scalar_t threshold = 3.84) {
        // Epipolar line in second image l = x1'F12 = [a b c]
        const scalar_t a = kp1.x() * F12(0, 0) + kp1.y() * F12(1, 0) + F12(2, 0);
        const scalar_t b = kp1.x() * F12(0, 1) + kp1.y() * F12(1, 1) + F12(2, 1);
        const scalar_t c = kp1.x() * F12(0, 2) + kp1.y() * F12(1, 2) + F12(2, 2);

        const scalar_t num = a * kp2.x() + b * kp2.y() + c;

        const scalar_t den = a * a + b * b;

        if (den == 0)
            return false;

        const scalar_t dsqr = num * num / den;

        return dsqr < threshold;
    }

    // From ORB SLAM 2
    inline CML::Matrix33 computeFundamental(const Camera &camera1, const Matrix33 &K1, const Camera &camera2, const Matrix33 &K2) {
        Matrix33 R1w = camera1.getRotationMatrix();
        Vector3 t1w = camera1.getTranslation();
        Matrix33 R2w = camera2.getRotationMatrix();
        Vector3 t2w = camera2.getTranslation();

        Matrix33 R12 = R1w * R2w.transpose();
        Vector3 t12 = -R1w * R2w.transpose() * t2w + t1w;

        Matrix33 t12x = skew(t12);
        return K1.transpose().inverse() * t12x * R12 * K2.inverse();
    }




}




#endif //CML_CAMERA_H
