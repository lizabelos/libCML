#ifndef CML_MATRIXPYRAMID_H
#define CML_MATRIXPYRAMID_H

#include <cml/config.h>
#include <cml/image/Array2D.h>

namespace CML {

    template <typename T> class MatrixPyramid {

    public:
        explicit MatrixPyramid(const Array2D<T> &matrix, int numLevels) {
            mNumLevels = numLevels;
            mMatrices = new Array2D<T>[numLevels];
            mMatrices[0] = matrix;
            for (int i = 1; i < mNumLevels; i++) {
                mMatrices[i] = mMatrices[i - 1].resize(0.5);
            }
        }

        ~MatrixPyramid() {
            delete[] mMatrices;
        }

        int getNumLevels() {
            return mNumLevels;
        }

        Array2D<T> &getMatrix(size_t level) {
            return mMatrices[level];
        }

        const Array2D<T> &getMatrix(size_t level) const {
            return mMatrices[level];
        }

    private:
        int mNumLevels;
        Array2D<T> *mMatrices;

    };

}

#endif