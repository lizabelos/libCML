#ifndef CML_EXPOSURE_H
#define CML_EXPOSURE_H

#include <cml/config.h>

namespace CML {

    /**
     * @brief A class to transform the color intensity of a frame to an other frame with a different exposition
     */
    class ExposureTransition {

        friend class Exposure;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        ExposureTransition() {

        }

        /**
         * @brief Apply the transformation to a color intensity
         * @param v The color intensity
         * @return The transformed color intensity
         */
        EIGEN_STRONG_INLINE scalar_t operator()(scalar_t v) const {
            return mA * v + mB;
        }

        /**
         * @return Return the parameters of the transformation
         */
        EIGEN_STRONG_INLINE Vector2 getParameters() const {
            return Vector2(mA, mB);
        }

    protected:
        EIGEN_STRONG_INLINE ExposureTransition(scalar_t a, scalar_t b) : mA(a), mB(b) {

        }

    private:
        scalar_t mA, mB;

    };

    /**
     * @brief This class represent the exposure of a frame
     */
    class Exposure {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        /**
         * @brief Initialize the exposure with default parameters
         */
        EIGEN_STRONG_INLINE Exposure() : mA(0), mB(0), mExposureFromCamera(1) {

        }

        /**
         * @brief Initialize the exposure with the exposure time of the frame
         * @param exposureFromCamera The exposure time of the frame
         */
        EIGEN_STRONG_INLINE Exposure(scalar_t exposureFromCamera) : mA(0), mB(0), mExposureFromCamera(exposureFromCamera) {
            assertThrow(exposureFromCamera > 0, "Invalid exposure");
        }

        /**
         * @brief nitialize the exposure with the exposure time of the frame, and two transformation parameters a and b
         * @param exposureFromCamera The exposure time of the frame
         * @param a The transformation parameters
         * @param b The transformation parameters
         */
        EIGEN_STRONG_INLINE Exposure(scalar_t exposureFromCamera, scalar_t a, scalar_t b) : mA(a), mB(b), mExposureFromCamera(exposureFromCamera) {
            assertThrow(exposureFromCamera > 0, "Invalid exposure");
        }

        /**
         * @return Return the transformation parameters
         */
        EIGEN_STRONG_INLINE Vector2 getParameters() const {
            return Vector2(mA, mB);
        }

        /**
         * @return Return the exposure time from the camera
         */
        EIGEN_STRONG_INLINE scalar_t getExposureFromCamera() const {
            return mExposureFromCamera;
        }

        /**
         * @brief Set the transformations parameters
         * @param other The transformations parameters as an Exposure
         */
        EIGEN_STRONG_INLINE void setParameters(const Exposure &other) {
            mA = other.mA;
            mB = other.mB;
        }

        /**
         * @brief Set the transformations parameters
         * @param a The transformations parameters as two scalar
         * @param b The transformations parameters as two scalar
         */
        EIGEN_STRONG_INLINE void setParameters(scalar_t a, scalar_t b) {
            mA = a;
            mB = b;
        }

        /**
         * @brief Compute the transformation between two exposure
         * @param other The other exposure
         * @return The transformation between the two exposure
         */
        EIGEN_STRONG_INLINE ExposureTransition to(const Exposure &other) const {
            scalar_t a = exp(other.mA - this->mA) * other.mExposureFromCamera / this->mExposureFromCamera;
            scalar_t b = other.mB - a * this->mB;
            return {a, b};
        }

        /**
         * @brief Add a value to a and b. Useful for gauss-newton optimisation.
         */
        EIGEN_STRONG_INLINE Exposure add(scalar_t a, scalar_t b) {
            Exposure e = *this;
            e.mA += a;
            e.mB += b;
            return e;
        }

        EIGEN_STRONG_INLINE Exposure& operator=(const Exposure &other) = delete;

        /**
         * @brief Copy the other exposure into this exposure
         * @param other The other exposure
         */
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