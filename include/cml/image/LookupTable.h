#ifndef CML_LOOKUPTABLE_H
#define CML_LOOKUPTABLE_H

#include <cml/config.h>

namespace CML {

    class GrayLookupTable {

    public:
        GrayLookupTable() {
            for (int i = 0; i < 256; i++) {
                mValues[i] = i;
                mInv[i] = i;
            }
        }

        GrayLookupTable(const Vectorf<256> &values) {
            mValues = values;
            computeInverse();
        }

        static GrayLookupTable fromInverse(const Vectorf<256> &values) {
            GrayLookupTable table(values);
            std::swap(table.mValues, table.mInv);
            return table;
        }

        static float gammaEncode(float v, scalar_t gamma = 2.2) {
            return pow((scalar_t)v / 255.0,1.0 / gamma) * 255.0;
        }

        static float gammaDecode(float v, scalar_t gamma = 2.2) {
            return pow((scalar_t)v / 255.0,gamma) * 255.0;
        }

        static double gammaEncode(double v, scalar_t gamma = 2.2) {
            return pow((scalar_t)v / 255.0,1.0 / gamma) * 255.0;
        }

        static double gammaDecode(double v, scalar_t gamma = 2.2) {
            return pow((scalar_t)v / 255.0,gamma) * 255.0;
        }

        static GrayLookupTable gammaEncode(scalar_t gamma = 2.2) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                values[i] = pow((scalar_t)i / 255.0,1.0 / gamma) * 255.0;
            }
            return GrayLookupTable(values);
        }

        static GrayLookupTable gammaDecode(scalar_t gamma = 2.2) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                values[i] = pow((scalar_t)i / 255.0,gamma) * 255.0;
            }
            return GrayLookupTable(values);
        }

        EIGEN_STRONG_INLINE float operator()(uint8_t input) const {
            return mValues[input];
        }

        EIGEN_STRONG_INLINE float inverse(uint8_t input) const {
            return mInv[input];
        }

        EIGEN_STRONG_INLINE float operator()(int input) const {
            return mValues[input];
        }

        EIGEN_STRONG_INLINE float inverse(int input) const {
            return mInv[input];
        }

        EIGEN_STRONG_INLINE float operator()(float input) const {
            uint8_t i0 = input;
            uint8_t i1 = i0 + 1;
            input -= i0;
            return mValues[i0] * (1.0f - input) + mValues[i1] * input;
        }

        EIGEN_STRONG_INLINE float inverse(float input) const {
            uint8_t i0 = input;
            uint8_t i1 = i0 + 1;
            input -= i0;
            return mInv[i0] * (1.0f - input) + mInv[i1] * input;
        }

    protected:
        void computeInverse() {
            // from dso
            for(int i=1;i<255;i++)
            {
                // find val, such that Binv[val] = i.
                // I dont care about speed for this, so do it the stupid way.

                for(int s=1;s<255;s++)
                {
                    if(mValues[s] <= i && mValues[s+1] >= i)
                    {
                        mInv[i] = s+(i - mValues[s]) / (mValues[s+1]-mValues[s]);
                        break;
                    }
                }
            }
            mInv[0] = 0;
            mInv[255] = 255;
        }

    private:
        Vectorf<256> mValues, mInv;

    };

}

#endif