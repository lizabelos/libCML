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
            //mValues = values;
            // do a memcpy
            memcpy((void*)mValues.data(), (void*)values.data(), sizeof(float) * 256);
            computeInverse();
        }

        static GrayLookupTable contrastAndBrightness(scalar_t contrast, scalar_t brightness) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                float value = (i - 127.5f) * contrast + 127.5f + brightness;
                if (value < 0) {
                    value = 0;
                } else if (value > 255) {
                    value = 255;
                }
                values[i] = value;
            }
            return GrayLookupTable(values);
        }

        static GrayLookupTable gamma(scalar_t gamma) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                float value = std::pow(i / 255.0f, gamma) * 255.0f;
                if (value < 0) {
                    value = 0;
                } else if (value > 255) {
                    value = 255;
                }
                values[i] = value;
            }
            return GrayLookupTable(values);
        }

        static GrayLookupTable level(scalar_t black, scalar_t white) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                float value = (i - black) / (white - black) * 255.0f;
                if (value < 0) {
                    value = 0;
                } else if (value > 255) {
                    value = 255;
                }
                values[i] = value;
            }
            return GrayLookupTable(values);
        }

        static GrayLookupTable level(scalar_t black, scalar_t white, scalar_t gamma) {
            Vectorf<256> values;
            for (int i = 0; i < 256; i++) {
                float value = (i - black) / (white - black) * 255.0f;
                value = std::pow(value / 255.0f, gamma) * 255.0f;
                if (value < 0) {
                    value = 0;
                } else if (value > 255) {
                    value = 255;
                }
                values[i] = value;
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
        //Vectorf<256> mValues, mInv;
        // use std array instead
        std::array<float, 256> mValues, mInv;
    };

}

#endif