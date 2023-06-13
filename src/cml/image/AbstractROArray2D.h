#ifndef CML_ABSTRACTROARRAY2D_H
#define CML_ABSTRACTROARRAY2D_H

#include <cml/config.h>

namespace CML {

    extern Atomic<size_t> __array2DCounter;

    template <typename T> class AbstractROArray2D  {

    public:
        AbstractROArray2D() : mId(__array2DCounter++) {

        }

        virtual ~AbstractROArray2D() {

        }

        EIGEN_STRONG_INLINE size_t getId() const {
            return mId;
        }

        EIGEN_STRONG_INLINE void renewId() {
            mId = __array2DCounter++;
        }

        virtual EIGEN_STRONG_INLINE int getWidth() const = 0;

        virtual EIGEN_STRONG_INLINE int getHeight() const = 0;

        virtual EIGEN_STRONG_INLINE T get(int x, int y) const = 0;

        template <typename V> EIGEN_STRONG_INLINE T get(Vector2i pos) const {
            return get(pos[0], pos[1]);
        }

        template <typename V> EIGEN_STRONG_INLINE T get(V pos) const {
            return get(pos[0], pos[1]);
        }

        EIGEN_STRONG_INLINE virtual T interpolate(const Vector2d &pos) const {

            scalar_t x = pos.x();
            scalar_t y = pos.y();

            int ix = (int)x;
            int iy = (int)y;
            scalar_t dx = x - (scalar_t)ix;
            scalar_t dy = y - (scalar_t)iy;
            scalar_t dxdy = dx * dy;

            T result = get(ix + 1, iy + 1) * dxdy
                       + get(ix + 0, iy + 1) * (dy-dxdy)
                       + get(ix + 1, iy + 0) * (dx-dxdy)
                       + get(ix + 0, iy + 0) * (1-dx-dy+dxdy);

            return result;

        }

        EIGEN_STRONG_INLINE virtual T interpolate(const Vector2f &pos) const {

            scalar_t x = pos.x();
            scalar_t y = pos.y();

            int ix = (int)x;
            int iy = (int)y;
            scalar_t dx = x - (scalar_t)ix;
            scalar_t dy = y - (scalar_t)iy;
            scalar_t dxdy = dx * dy;

            T result = get(ix + 1, iy + 1) * dxdy
                       + get(ix + 0, iy + 1) * (dy-dxdy)
                       + get(ix + 1, iy + 0) * (dx-dxdy)
                       + get(ix + 0, iy + 0) * (1-dx-dy+dxdy);

            return result;

        }

        EIGEN_STRONG_INLINE virtual T interpolateBilinear(const Vector2 &pos) const {
            scalar_t x = pos.x();
            scalar_t y = pos.y();

            int ix = (int)x;
            int iy = (int)y;

            T tl = get(ix + 0, iy + 0);
            T tr = get(ix + 1, iy + 0);
            T bl = get(ix + 0, iy + 1);
            T br = get(ix + 1, iy + 1);

            scalar_t dx = x - ix;
            scalar_t dy = y - iy;
            T leftInt = bl * dy + tl * (1 - dy);
            T rightInt = br * dy + tr * (1 - dy);

            return rightInt * dx + leftInt * (1 - dx);
        }

    private:
        size_t mId;

    };

}

#endif