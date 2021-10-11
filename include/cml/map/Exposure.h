#ifndef CML_EXPOSURE_H
#define CML_EXPOSURE_H

#include <cml/config.h>

namespace CML {

    class ExposureTransition {

        friend class Exposure;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        ExposureTransition() {

        }

        EIGEN_STRONG_INLINE scalar_t operator()(scalar_t v) const {
            return mA * v + mB;
        }

        EIGEN_STRONG_INLINE Vector2 getParameters() const {
            return Vector2(mA, mB);
        }

    protected:
        EIGEN_STRONG_INLINE ExposureTransition(scalar_t a, scalar_t b) : mA(a), mB(b) {

        }

    private:
        scalar_t mA, mB;

    };

    class Exposure {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        EIGEN_STRONG_INLINE Exposure() : mA(0), mB(0), mExposureFromCamera(1) {

        }

        EIGEN_STRONG_INLINE Exposure(scalar_t exposureFromCamera) : mA(0), mB(0), mExposureFromCamera(exposureFromCamera) {
            assertThrow(exposureFromCamera > 0, "Invalid exposure");
        }

        EIGEN_STRONG_INLINE Exposure(scalar_t exposureFromCamera, scalar_t a, scalar_t b) : mA(a), mB(b), mExposureFromCamera(exposureFromCamera) {
            assertThrow(exposureFromCamera > 0, "Invalid exposure");
        }

        EIGEN_STRONG_INLINE Vector2 getParameters() const {
            return Vector2(mA, mB);
        }

        EIGEN_STRONG_INLINE scalar_t getExposureFromCamera() const {
            return mExposureFromCamera;
        }

        EIGEN_STRONG_INLINE void setParameters(const Exposure &other) {
            mA = other.mA;
            mB = other.mB;
        }

        EIGEN_STRONG_INLINE void setParameters(scalar_t a, scalar_t b) {
            mA = a;
            mB = b;
        }

        EIGEN_STRONG_INLINE ExposureTransition to(const Exposure &other) const {
            scalar_t a = exp(other.mA - this->mA) * other.mExposureFromCamera / this->mExposureFromCamera;
            scalar_t b = other.mB - a * this->mB;
            return {a, b};
        }

        EIGEN_STRONG_INLINE Exposure add(scalar_t a, scalar_t b) {
            Exposure e = *this;
            e.mA += a;
            e.mB += b;
            return e;
        }

        EIGEN_STRONG_INLINE Exposure& operator=(const Exposure &other) = delete;

        EIGEN_STRONG_INLINE void setParametersAndExposure(const Exposure &other) {
            mA = other.mA;
            mB = other.mB;
            mExposureFromCamera = other.mExposureFromCamera;
        }

    private:
        scalar_t mA, mB;
        scalar_t mExposureFromCamera;

    };

    inline std::ostream& operator<<(std::ostream& os, const Exposure &exposure) {
        os << "[Exposure](" << exposure.getParameters()(0) << " ; " << exposure.getParameters()(1) << " ; " << exposure.getExposureFromCamera() << ")";
        return os;
    }

    inline std::ostream& operator<<(std::ostream& os, const ExposureTransition &exposure) {
        os << "[ExposureTransition](" << exposure.getParameters()(0) << " ; " << exposure.getParameters()(1) << ")";
        return os;
    }

}

#endif