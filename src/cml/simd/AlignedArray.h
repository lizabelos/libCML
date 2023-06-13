#ifndef CML_ALIGNEDARRAY
#define CML_ALIGNEDARRAY

#include <cml/config.h>
#include <cml/simd/M128.h>

namespace CML {

    template <typename T, int b> class AlignedArray {

    public:
        EIGEN_STRONG_INLINE AlignedArray(size_t size) {
            assertThrow(size > 0, "Invalid size : " + std::to_string(size));
            const int padT = 1 + ((1 << b)/sizeof(T));
            mPtr = new T[size + padT];
            mAlignedPtr = (T*)(( ((uintptr_t)(mPtr+padT)) >> b) << b);
            mSize = size;
        }

        AlignedArray(const AlignedArray &other) = delete;

        AlignedArray& operator=(const AlignedArray &other) = delete;

        EIGEN_STRONG_INLINE ~AlignedArray() {
            delete mPtr;
        }

        EIGEN_STRONG_INLINE T& operator [](size_t i) {
            assertThrow(i < mSize, "Out of range : " + std::to_string(i) + "/" + std::to_string(mSize));
            return mAlignedPtr[i];
        }

        EIGEN_STRONG_INLINE T* p() {
            return mAlignedPtr;
        }

        EIGEN_STRONG_INLINE T* p(size_t i) {
            return &mAlignedPtr[i];
        }

        EIGEN_STRONG_INLINE M128 m128(size_t i) {
            assertThrow(i % 4 == 0, "You are trying to make a M128 with unaligned index");
            return M128(p(i));
        }

    private:
        size_t mSize;
        T *mPtr, *mAlignedPtr;

    };

}

#endif