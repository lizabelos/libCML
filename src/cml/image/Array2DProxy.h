//
// Created by thomas on 13/10/2020.
//

#ifndef CML_ARRAY2DPROXY_H
#define CML_ARRAY2DPROXY_H

#include <cml/config.h>
#include <cml/image/AbstractROArray2D.h>
#include <cml/image/Array2D.h>

namespace CML {

    class GrayImageProxy : public AbstractROArray2D<float> {

    public:
        EIGEN_STRONG_INLINE GrayImageProxy(const GrayImage &children, const GrayLookupTable &lut, const Array2D<float> &vignette, const float &maxValue)
        : AbstractROArray2D<float>(), mChildren(children), mLut(lut), mVignette(vignette), mMaxValue(maxValue)
        {

        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE float get(int x, int y) const final {
            scalar_t r = 1.0f / (mVignette.get((float)x * (float)mVignette.getWidth() / (float)getWidth(), (float)y * (float)mVignette.getHeight() / (float)getHeight()) / mMaxValue);
            return mLut(mChildren.get(x, y)) * r;
        }

        EIGEN_STRONG_INLINE const GrayImage &children() const {
            return mChildren;
        }

    private:
        const GrayImage &mChildren;

        const GrayLookupTable &mLut;

        const Array2D<float> &mVignette;
        const float &mMaxValue;

    };

    class FloatImageProxy : public AbstractROArray2D<float> {

    public:
        EIGEN_STRONG_INLINE FloatImageProxy(const FloatImage &children, const GrayLookupTable &lut, const Array2D<float> &vignette, const float &maxValue)
                : AbstractROArray2D<float>(), mChildren(children), mLut(lut), mVignette(vignette), mMaxValue(maxValue)
        {

        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE float get(int x, int y) const final {
            scalar_t r = 1.0f / (mVignette.get((float)x * (float)mVignette.getWidth() / (float)getWidth(), (float)y * (float)mVignette.getHeight() / (float)getHeight()) / mMaxValue);
            return mLut(mChildren.get(x, y)) * r;
        }

        EIGEN_STRONG_INLINE const FloatImage &children() const {
            return mChildren;
        }

    private:
        const FloatImage &mChildren;

        const GrayLookupTable &mLut;

        const Array2D<float> &mVignette;
        const float &mMaxValue;

    };

    class FastFloatImageProxy : public AbstractROArray2D<float> {

    public:
        EIGEN_STRONG_INLINE FastFloatImageProxy(const FloatImage &children, const GrayLookupTable &lut, const Array2D<float> &vignette, const float &maxValue)
                : AbstractROArray2D<float>(), mChildren(children), mLut(lut), mVignette(vignette), mMaxValue(maxValue)
        {
            assertThrow(children.getWidth() == vignette.getWidth() && children.getHeight() == vignette.getHeight(), "The size need to be the same !");

        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE float get(int x, int y) const final {
            scalar_t r = 1.0f / (mVignette.get(x, y) / mMaxValue);
            return mLut(mChildren.get(x, y)) * r;
        }

        EIGEN_STRONG_INLINE const FloatImage &children() const {
            return mChildren;
        }

    private:
        const FloatImage &mChildren;

        const GrayLookupTable &mLut;

        const Array2D<float> &mVignette;
        const float &mMaxValue;

    };


    class ImageProxy : public AbstractROArray2D<ColorRGBA> {

    public:
        EIGEN_STRONG_INLINE ImageProxy(const Image &children, const GrayLookupTable &lut, const Array2D<float> &vignette, const float &maxValue)
                : AbstractROArray2D<ColorRGBA>(), mChildren(children), mLut(lut), mVignette(vignette), mMaxValue(maxValue)
        {

        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE ColorRGBA get(int x, int y) const final {
            ColorRGBA c = mChildren.get(x, y);
            scalar_t r = 1.0f / (mVignette.get(x, y) / mMaxValue);
            return ColorRGBA(mLut(c.r()), mLut(c.g()), mLut(c.b())) * r;
        }

        EIGEN_STRONG_INLINE const Image &children() const {
            return mChildren;
        }

    private:
        const Image &mChildren;

        const GrayLookupTable &mLut;

        const Array2D<float> &mVignette;
        const float &mMaxValue;

    };

    class FastImageProxy : public AbstractROArray2D<ColorRGBA> {

    public:
        EIGEN_STRONG_INLINE FastImageProxy(const Image &children, const GrayLookupTable &lut, const Array2D<float> &vignette, const float &maxValue)
                : AbstractROArray2D<ColorRGBA>(), mChildren(children), mLut(lut), mVignette(vignette), mMaxValue(maxValue)
        {
            assertThrow(children.getWidth() == vignette.getWidth() && children.getHeight() == vignette.getHeight(), "The size need to be the same !");
        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE ColorRGBA get(int x, int y) const final {
            ColorRGBA c = mChildren.get(x, y);
            scalar_t r = 1.0f / (mVignette.get((float)x * (float)mVignette.getWidth() / (float)getWidth(), (float)y * (float)mVignette.getHeight() / (float)getHeight()) / mMaxValue);
            return ColorRGBA(mLut(c.r()), mLut(c.g()), mLut(c.b())) * r;
        }

        EIGEN_STRONG_INLINE const Image &children() const {
            return mChildren;
        }

    private:
        const Image &mChildren;

        const GrayLookupTable &mLut;

        const Array2D<float> &mVignette;
        const float &mMaxValue;

    };

    class WeightedGradientImageProxy : public AbstractROArray2D<float> {

    public:
        EIGEN_STRONG_INLINE WeightedGradientImageProxy(const GradientImage &children, GrayLookupTable *lut) : AbstractROArray2D<float>(), mChildren(children), mLut(lut)
        {

        }

        EIGEN_STRONG_INLINE int getWidth() const final {
            return mChildren.getWidth();
        }

        EIGEN_STRONG_INLINE int getHeight() const final {
            return mChildren.getHeight();
        }

        EIGEN_STRONG_INLINE float get(int x, int y) const final {
            const auto &colorAndGrad = mChildren.get(x, y);
            float gw = grad(colorAndGrad[0]);
            return colorAndGrad.tail<2>().squaredNorm() * gw * gw;
        }

    protected:
        EIGEN_STRONG_INLINE float grad(float color) const {
            int c = lroundf(color);
            if(c<5) c=5;
            if(c>250) c=250;
            return mLut->inverse(c+1) - mLut->inverse(c);
        }

    private:
        const GradientImage &mChildren;
        GrayLookupTable *mLut;

    };


}

#endif //CML_ALL_ARRAY2DPROXY_H
