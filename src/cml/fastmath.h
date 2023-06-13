#ifndef CML_FASTMATH
#define CML_FASTMATH

namespace CML {

extern float fast_cos_values[];;
inline float fast_cos(float v) {
    return fast_cos_values[(int)(v / 60000.000000)];
}
extern float fast_sin_values[];;
inline float fast_sin(float v) {
    return fast_sin_values[(int)(v / 60000.000000)];
}
extern float fast_acos_values[];;
inline float fast_acos(float v) {
    return fast_acos_values[(int)(v / 60000.000000)];
}
extern float fast_asin_values[];;
inline float fast_asin(float v) {
    return fast_asin_values[(int)(v / 60000.000000)];
}

}

#endif
