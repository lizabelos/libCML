//
// Created by tbelos on 19/04/19.
//

#ifndef CML_MATRIX_H
#define CML_MATRIX_H

#include <cml/config.h>
#include <cml/image/LookupTable.h>
#include <cml/image/AbstractROArray2D.h>

#include <vector>
#include <memory>
#include <cmath>
#include <mutex>
#include <Eigen/Dense>

namespace CML {


    template<typename T>
    class Array2D : public AbstractROArray2D<T> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Array2D() : AbstractROArray2D<T>() {
            mData = nullptr;
        }

        Array2D(int width, int height) : AbstractROArray2D<T>() {
            mMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(width, height);
            mData = mMatrix.data();
        }

        Array2D(int width, int height, const T &fill) : AbstractROArray2D<T>() {
            mMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(width, height);
            for (int y = 0; y < height; y++) for (int x = 0; x < width; x++) {
                mMatrix(x,y) = fill;
            }
            mData = mMatrix.data();
            // mMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(width, height, fill);
        }

        Array2D(const Array2D<T> &other) : AbstractROArray2D<T>() {
            mMatrix = other.mMatrix;
            mData = mMatrix.data();
        }

        Array2D(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &other) : AbstractROArray2D<T>() {
            mMatrix = other;
            mData = mMatrix.data();
        }

        Array2D(int width, int height, T* data) : AbstractROArray2D<T>() {
            mMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>(width, height);
            memcpy(mMatrix.data(), data, sizeof(T) * width * height);
            mData = mMatrix.data();
        }

        static Array2D<T> from(const AbstractROArray2D<T> &other) {
            Array2D<T> result(other.getWidth(), other.getHeight());
            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 0; y < other.getHeight(); y++) {
                for (int x = 0; x < other.getWidth(); x++) {
                    result(x, y) = other.get(x, y);
                }
            }
            return result;
        }

        static void from(const AbstractROArray2D<T> &other, Array2D<T> &result) {
            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 0; y < other.getHeight(); y++) {
                for (int x = 0; x < other.getWidth(); x++) {
                    result(x, y) = other.get(x, y);
                }
            }
        }

        Array2D& operator=(const Array2D<T> &other) {
            mMatrix = other.mMatrix;
            mData = mMatrix.data();
            return *this;
        }

        Array2D& copyToThis(const Array2D<T> &other) {
            mMatrix = other.mMatrix;
            mData = mMatrix.data();
            return *this;
        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mMatrix.rows();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mMatrix.cols();
        }

        bool isEmpty() const {
            return mMatrix.size() == 0;
        }

        Array2D<T> transpose() const {
            return Array2D<T>(mMatrix.transpose());
        }

        T* data() {
            return mMatrix.data();
        }

        const T* data() const {
            return mMatrix.data();
        }

        const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &eigenMatrix() const {
            return mMatrix;
        }

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &eigenMatrix() {
            return mMatrix;
        }

        EIGEN_STRONG_INLINE T &operator()(int x, int y) {
            return mMatrix(x, y);
        }

        EIGEN_STRONG_INLINE T &operator()(const Eigen::Vector2i &pos) {
            return mMatrix(pos.x(), pos.y());
        }

        EIGEN_STRONG_INLINE void set(T value, int x, int y) {
            mMatrix(x, y) = value;
        }

        EIGEN_STRONG_INLINE void set(T value, const Eigen::Vector2i &pos) {
            mMatrix(pos.x(), pos.y()) = value;
        }

        EIGEN_STRONG_INLINE const T &operator()(int x, int y) const {
            return mData[y * getWidth() + x];
        }

        EIGEN_STRONG_INLINE const T &operator()(const Eigen::Vector2i &pos) const {
            return mData[pos.y() * getWidth() + pos.x()];
        }

        EIGEN_STRONG_INLINE T get(int x, int y) const final {
            return mData[y * getWidth() + x];
        }

        EIGEN_STRONG_INLINE T interpolate(const Vector2f &pos) const override {

            const float x = pos.x();
            const float y = pos.y();

            const int ix = (int)x;
            const int iy = (int)y;
            const float dx = x - (float)ix;
            const float dy = y - (float)iy;
            const float dxdy = dx * dy;

            const int w = getWidth();

            const int i1 = iy * w + ix;
            const int i2 = i1 + w;

            return   mData[i1 + 0] * (1-dx-dy+dxdy)
                   + mData[i1 + 1] * (dx-dxdy)
                   + mData[i2 + 0] * (dy-dxdy)
                   + mData[i2 + 1] * dxdy;

        }

        EIGEN_STRONG_INLINE Vector3f gradient(int x, int y) const {
            return Vector3f(
                    (float)get(x, y),
                    ((float) get(x + 1, y) - (float) get(x - 1, y)) * 0.5f,
                    ((float) get(x, y + 1) - (float) get(x, y - 1)) * 0.5f
            );
        }

        EIGEN_STRONG_INLINE Vector3f gradient(const Eigen::Vector2i &pos) const {
            return gradient(pos.x(), pos.y());
        }

        EIGEN_STRONG_INLINE GradientImage gradientImage() const {
            GradientImage output(getWidth(), getHeight(), Vector3f(0, 0, 0));

            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 1; y < getHeight() - 1; y++) {
                for (int x = 1; x < getWidth() - 1; x++) {
                    output(x, y) = gradient(x, y);
                }
            }
            return output;
        }

        EIGEN_STRONG_INLINE void gradientImage(GradientImage &output) const {
            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 1; y < getHeight() - 1; y++) {
                for (int x = 1; x < getWidth() - 1; x++) {
                    output(x, y) = gradient(x, y);
                }
            }
        }

        EIGEN_STRONG_INLINE Array2D<T> operator*(T other) const {
            Array2D<T> result = *this;
            result.mMatrix *= other;
            return result;
        }

        EIGEN_STRONG_INLINE Array2D<T> operator/(T other) const {
            Array2D<T> result = *this;
            result.mMatrix /= other;
            return result;
        }

        EIGEN_STRONG_INLINE Array2D<T> operator+(T other) const {
            Array2D<T> result = *this;
            result.mMatrix = result.mMatrix.array() + other;
            return result;
        }

        EIGEN_STRONG_INLINE Array2D<T> operator-(T other) const {
            Array2D<T> result = *this;
            result.mMatrix = result.mMatrix.array() - other;
            return result;
        }

        EIGEN_STRONG_INLINE Array2D<T> operator+(const Array2D<T> &other) const {
            Array2D<T> result = *this;
            result.mMatrix += other.mMatrix;
            return result;
        }

        EIGEN_STRONG_INLINE Array2D<T> operator-(const Array2D<T> &other) const {
            Array2D<T> result = *this;
            result.mMatrix -= other.mMatrix;
            return result;
        }

        [[nodiscard]] Array2D<T> resize(int newWidth, int newHeight) const;
        /*
        [[nodiscard]] Array2D<T> resize(int newWidth, int newHeight) const {
            assertThrow(newWidth > 0 && newWidth < 10000 && newHeight > 0 && newHeight < 10000,
                    "Invalid resize of (" + std::to_string(newWidth) + "x" + std::to_string(newHeight) + ")");

            Array2D<T> result(newWidth, newHeight);
            stbir_resize_uint8((const unsigned char *)mMatrix.data(), getWidth(), getHeight(), 0, (unsigned char*)result.data(), result.getWidth(), result.getHeight(), 0, sizeof(T));
            return result;
        }*/

        [[nodiscard]] Array2D<T> resize(float ratio) const {
            assertThrow(ratio > 0 && ratio < 10, "Invalid resize of ratio " + std::to_string(ratio));
            Array2D<T> result = resize((float)getWidth() * ratio, (float)getHeight() * ratio);
            return result;
        }

        [[nodiscard]] Array2D<T> reduceByTwo() const {
            //assertThrow(getWidth() % 2 == 0 && getHeight() % 2 == 0, "Size must be divisable by 2");
            int newWidth = getWidth() / 2, newHeight = getHeight() / 2;
            Array2D<T> result(newWidth, newHeight);
            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 0; y < newHeight; y++) {
                for (int x = 0; x < newWidth; x++) {
                    result(x, y) = (get(x * 2, y * 2) + get(x * 2 + 1, y * 2) + get(x * 2, y * 2 + 1) + get(x * 2 + 1, y * 2 + 1)) / T(4);
                }
            }
            return result;
        }

        [[nodiscard]] Array2D<T> fastReduceByTwo() const {
            //assertThrow(getWidth() % 2 == 0 && getHeight() % 2 == 0, "Size must be divisable by 2");
            int newWidth = getWidth() / 2, newHeight = getHeight() / 2;
            Array2D<T> result(newWidth, newHeight);
            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int y = 0; y < newHeight; y++) {
                for (int x = 0; x < newWidth; x++) {
                    result(x, y) = get(x * 2, y * 2);
                }
            }
            return result;
        }

        Array2D<T> crop(int startX, int startY, int newWidth, int newHeight) const {
            Array2D<T> result(newWidth, newHeight);
            result.mMatrix = mMatrix.block(startX, startY, newWidth, newHeight);
            return result;
        }

        [[nodiscard]] Array2D<T> verticalFlip() const {
            return Array2D<T>(mMatrix.colwise().reverse());
        }

        [[nodiscard]] Array2D<T> horizontalFlip() const {
            return Array2D<T>(mMatrix.rowwise().reverse());
        }

        Array2D<T> convolution(const Array2D<T> &kernel) const;

        Array2D<T> blur() const {

            Array2D<T> newImage(*this);

            int KSizeX = 3;
            int KSizeY = 3;

            int limitRow = newImage.mMatrix.rows() - KSizeX;
            int limitCol = newImage.mMatrix.cols() - KSizeY;

            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for ( int col = KSizeY; col < limitCol; col++ ) {
                for (int row = KSizeX; row < limitRow; row++) {

                    newImage.mMatrix(row,col) = (mMatrix(row + 0, col + 0) +
                                                mMatrix(row + 0, col + 1) +
                                                mMatrix(row + 0, col - 1) +
                                                mMatrix(row + 1, col + 0) +
                                                mMatrix(row + 1, col + 1) +
                                                mMatrix(row + 1, col - 1) +
                                                mMatrix(row - 1, col + 0) +
                                                mMatrix(row - 1, col + 1) +
                                                mMatrix(row - 1, col - 1)) / 9.0f;

                }
            }

            return newImage;
        }

        template<typename U>
        Array2D<U> cast() const {
            Array2D<U> result(mMatrix.template cast<U>());
            return result;
        }

        [[nodiscard]] Array2D<float> toGrayImage() const {
            return Array2D<float>(mMatrix.unaryExpr([](const T& elem) -> float {
                return elem.gray();
            }));
        }

        Array2D<float> applyLut(const GrayLookupTable &lut) const {
            return Array2D<float>(mMatrix.unaryExpr([lut](const T& elem) -> float {
                return lut(elem);
            }));
        }

        Array2D<float> removeVignette(Array2D<float> v, float maxValue) const {
            Array2D<float> output(getWidth(), getHeight(), 0.0f);

            #if CML_USE_OPENMP
            #pragma omp parallel for collapse(2) schedule(static)
            #endif
            for (int x = 0; x < getWidth(); x++) {
                for (int y = 0; y < getHeight(); y++) {
                    if (v(x,y) != 0) {
                        output(x, y) = (float)get(x,y) / (v(x,y) / maxValue);
                    }
                }
            }
            return output;
        }

        T min() const {
            return mMatrix.minCoeff();
        }

        T max() const {
            return mMatrix.maxCoeff();
        }

        void normalize() {
            T m = max();
            const int wh = getWidth() * getHeight();
            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int i = 0; i < wh; i++) {
                mMatrix.data()[i] = mMatrix.data()[i] / m;
            }
        }

        void elementWiseInverse() {
            const int wh = getWidth() * getHeight();
            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int i = 0; i < wh; i++) {
                mMatrix.data()[i] = T(1) / mMatrix.data()[i];
            }
        }

        Array2D<T> clip(T min = 0, T max = 255) const {
            Array2D<T> result(getWidth(), getHeight());
            const int wh = getWidth() * getHeight();
            #if CML_USE_OPENMP
            #pragma omp parallel for schedule(static)
            #endif
            for (int i = 0; i < wh; i++) {
                if (data()[i] > max) {
                    result.data()[i] = max;
                } else if (data()[i] < min) {
                    result.data()[i] = min;
                } else {
                    result.data()[i] = data()[i];
                }
            }
            return result;
        }

        /*Array2D<T> autoadjustAndClip() const {
            T lower = min();
            T higher = max();
            return (((*this) - lower) * T(255) / higher).clip(0, 255);
        }*/

        void drawCenteredRect(Vector2 pos, int radius, T value) {
            for (int i = 0; i < radius; i++) {
                for (int j = 0; j < radius; j++) {
                    (*this)(pos.x() + i, pos.x() + j) = value;
                    (*this)(pos.x() - i, pos.x() + j) = value;
                    (*this)(pos.x() + i, pos.x() - j) = value;
                    (*this)(pos.x() - i, pos.x() - j) = value;
                }
            }
        }

        void drawLine(Vector2 from, Vector2 to, T value) {
            int numStep = (from - to).cwiseAbs().maxCoeff();
            for (int i = 0; i < numStep; i++) {
                Vector2 pos = (from * (scalar_t)(i) + to * (scalar_t)(numStep - i)) / (scalar_t)numStep;
                drawCenteredRect(pos, 5, value);
            }
        }


        void saveBmp(const std::string &path) const {
            logger.info("Saving image to " + path + ". width = " + std::to_string(getWidth()) + "; height = " + std::to_string(getHeight()) + "; channels = " + std::to_string(sizeof(T)));

            int w = getWidth();
            int h = getHeight();

            FILE *f;
            unsigned char *img = NULL;
            int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int

            img = (unsigned char *)malloc(3*w*h);
            memset(img,0,3*w*h);

            for(int i=0; i<w; i++)
            {
                for(int j=0; j<h; j++)
                {
                    int x = i;
                    int y = (h - 1) - j;
                    ColorRGBA color = get(i, j);
                    img[(x+y*w)*3+2] = (unsigned char)(color.r());
                    img[(x+y*w)*3+1] = (unsigned char)(color.g());
                    img[(x+y*w)*3+0] = (unsigned char)(color.b());
                }
            }

            unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
            unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
            unsigned char bmppad[3] = {0,0,0};

            bmpfileheader[ 2] = (unsigned char)(filesize    );
            bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
            bmpfileheader[ 4] = (unsigned char)(filesize>>16);
            bmpfileheader[ 5] = (unsigned char)(filesize>>24);

            bmpinfoheader[ 4] = (unsigned char)(w    );
            bmpinfoheader[ 5] = (unsigned char)(w>> 8);
            bmpinfoheader[ 6] = (unsigned char)(w>>16);
            bmpinfoheader[ 7] = (unsigned char)(w>>24);
            bmpinfoheader[ 8] = (unsigned char)(h    );
            bmpinfoheader[ 9] = (unsigned char)(h>> 8);
            bmpinfoheader[10] = (unsigned char)(h>>16);
            bmpinfoheader[11] = (unsigned char)(h>>24);

            f = fopen(path.c_str(),"wb");
            fwrite(bmpfileheader,1,14,f);
            fwrite(bmpinfoheader,1,40,f);
            for(int i=0; i<h; i++)
            {
                fwrite(img+(w*(h-i-1)*3),3,w,f);
                fwrite(bmppad,1,(4-(w*3)%4)%4,f);
            }

            free(img);
            fclose(f);

        }

    private:
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mMatrix;
        T *mData;

    };

    Image loadImage(std::string fileName);

    GrayImage loadGrayImage(std::string fileName);

    FloatImage loadTiffImage(const uint8_t *data, size_t lenght);




}


#endif //CML_MATRIX_H
