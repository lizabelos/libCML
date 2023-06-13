#ifndef CML_FAST_H
#define CML_FAST_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/capture/CaptureImage.h>

namespace CML::Features {

    typedef enum FASTType {
        FAST_9 = 9,
        FAST_10 = 10,
        FAST_11 = 11,
        FAST_12 = 12
    } FASTType;

    class FAST {

        typedef struct { int x, y; } xy;
        typedef unsigned char byte;

    public:
        FAST() {

        }

        void compute(const GrayImage &frame, List<Corner> &corners, int threshold);

    private:
        int fast9_corner_score(const byte* p, const int pixel[], int bstart);
        void fast9_detect(const byte* im, int xsize, int ysize, int stride, int b, int* ret_num_corners);
        void fast9_score(const byte* i, int stride, int num_corners, int b);
        void nonmax_suppression(int num_corners, int* ret_num_nonmax, List<Corner> &points);

        int rsize = 8192;
        List<xy> ret_corners;
        List<int> scores;


    };

}

#endif