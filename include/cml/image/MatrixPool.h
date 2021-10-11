#ifndef CML_MATRIXPOOL_H
#define CML_MATRIXPOOL_H

#include <cml/config.h>
#include <cml/image/Array2D.h>

namespace CML {

    template <typename T> class MatrixPool {

    public:
        MatrixPool(int width, int height, int num) {
            assertThrow(num > 0, "Matrix Pool number lower than 1");
            assertThrow(width > 0 && height > 0, "Matrix pool invalid size");

            mNum = num;
            mElementSize = width * height;
            mSize = mElementSize * num;
            for (int i = 0; i < mNum; i++) {
                mMatrices.emplace_back(new Array2D<T>(width, height, T()));
            }
        }

        ~MatrixPool() {
            for (int i = 0; i < mNum; i++) {
                delete mMatrices[i].p();
            }
            // std::free((void*)mData);
        }

        Ptr<Array2D<T>, NonNullable> load() {
            LockGuard lg(mLoadMutex);
            size_t i = mCurrentMatrix;
            mCurrentMatrix = (mCurrentMatrix + 1) % mNum;
            mMatrices[i]->renewId();
            return mMatrices[i];
        }

        inline int getPoolSize() const {
            return mNum;
        }

    private:
        int mSize;
        int mElementSize;
        int mNum;
        // T *mData;
        List<Ptr<Array2D<T>, NonNullable>> mMatrices;
        Mutex mLoadMutex;
        int mCurrentMatrix = 0;
    };

}

#endif