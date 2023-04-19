#ifndef CML_SIMD_M128
#define CML_SIMD_M128

#include <cml/config.h>

namespace CML {

    class M128 {

    public:
        EIGEN_STRONG_INLINE M128(float a) : mValue(_mm_set1_ps(a)) {

        }

        EIGEN_STRONG_INLINE M128(float a, float b, float c, float d) : mValue(_mm_setr_ps(a, b, c, d)) {

        }

        EIGEN_STRONG_INLINE M128(float *p) : mValue(_mm_load_ps(p)) {

        }

        EIGEN_STRONG_INLINE M128 operator+(const M128 &b) const {
            return {_mm_add_ps(this->mValue, b.mValue)};
        }

        EIGEN_STRONG_INLINE M128 operator-(const M128 &b) const {
            return {_mm_sub_ps(this->mValue, b.mValue)};
        }

        EIGEN_STRONG_INLINE M128 operator*(const M128 &b) const {
            return {_mm_mul_ps(this->mValue, b.mValue)};
        }

        EIGEN_STRONG_INLINE const __m128 &data() const {
            return mValue;
        }

    protected:
        EIGEN_STRONG_INLINE M128(__m128 value) : mValue(value) {

        }

    private:
        __m128 mValue;

    };

}

#endif
