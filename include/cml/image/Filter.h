#ifndef CML_FILTER_H
#define CML_FILTER_H

#include <cml/config.h>
#include <cml/image/Array2D.h>

namespace CML {

    namespace Filter {

        EIGEN_STRONG_INLINE FloatImage verticalSobel() {
            FloatImage sobel(3, 3, 0.0f);
            sobel(0, 0) = -1;
            sobel(0, 1) = -2;
            sobel(0, 2) = -1;
            sobel(2, 0) = 1;
            sobel(2, 1) = 2;
            sobel(2, 2) = 1;
            return sobel / 8.0f;
        }

        EIGEN_STRONG_INLINE FloatImage horizontalSobel() {
            FloatImage sobel(3, 3, 0.0f);
            sobel(0, 0) = -1;
            sobel(1, 0) = -2;
            sobel(2, 0) = -1;
            sobel(0, 2) = 1;
            sobel(1, 2) = 2;
            sobel(2, 2) = 1;
            return sobel / 8.0f;
        }

        EIGEN_STRONG_INLINE FloatImage laplacian() {
            FloatImage laplacian(3,3,0.0f);
            laplacian(0,1) = 1;
            laplacian(1,0) = 1;
            laplacian(2,1) = 1;
            laplacian(1,2) = 1;
            laplacian(1,1) = -4;
            return laplacian / 8.0f;
        }

        // Inspired by the OpenCV implementation https://github.com/egonSchiele/OpenCV/blob/master/modules/imgproc/src/smooth.cpp
        EIGEN_STRONG_INLINE FloatImage gaussian1d(int n, scalar_t sigma) {
            const int SMALL_GAUSSIAN_SIZE = 7;
            static const float small_gaussian_tab[][SMALL_GAUSSIAN_SIZE] =
                    {
                            {1.f},
                            {0.25f, 0.5f, 0.25f},
                            {0.0625f, 0.25f, 0.375f, 0.25f, 0.0625f},
                            {0.03125f, 0.109375f, 0.21875f, 0.28125f, 0.21875f, 0.109375f, 0.03125f}
                    };
            const float* fixed_kernel = n % 2 == 1 && n <= SMALL_GAUSSIAN_SIZE && sigma <= 0 ? small_gaussian_tab[n>>1] : 0;

            FloatImage kernel(n, 1);
            float* c = (float*)kernel.data();

            double sigmaX = sigma > 0 ? sigma : ((n-1)*0.5 - 1)*0.3 + 0.8;
            double scale2X = -0.5/(sigmaX*sigmaX);
            double sum = 0;

            int i;
            for( i = 0; i < n; i++ )
            {
                double x = i - (n-1)*0.5;
                double t = fixed_kernel ? (double)fixed_kernel[i] : std::exp(scale2X*x*x);
                c[i] = t;
                sum += c[i];
            }

            sum = 1./sum;
            for( i = 0; i < n; i++ )
            {
                c[i] *= sum;
            }

            return kernel;

        }

        EIGEN_STRONG_INLINE FloatImage gaussian(int sx, int sy, scalar_t sigmaX, scalar_t sigmaY)
        {
            FloatImage xg = gaussian1d(sx, sigmaX), yg;
            if (sx == sy && sigmaX == sigmaY) {
                yg = xg;
            } else {
                yg = gaussian1d(sy, sigmaY);
            }

            FloatImage kernel(sx, sy);
            for (int y = 0; y < sy; y++) {
                for (int x = 0; x < sx; x++) {
                    kernel(x, y) = xg(x, 0) * yg(y, 0);
                }
            }

            return kernel;
        }

        EIGEN_STRONG_INLINE FloatImage averaging(int height, int width) {
            FloatImage kernel(width, height, 1.0f / (float)(width * height));
            return kernel;
        }

    }

}

#endif