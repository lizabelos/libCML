extern "C" {
    #include <fast/fast.h>
}

#include "cml/features/corner/FAST.h"

void CML::Features::FAST::compute(const GrayImage &frame, List<Corner> &points, int threshold, FASTType type , bool nonmaxSuppression) {

    xy* corners = nullptr;
    int num_corners;
    int* scores = nullptr;

    switch (type) {
        case FAST_9:
            corners = fast9_detect((const byte*)frame.data(), frame.getWidth(), frame.getHeight(), frame.getWidth(), threshold, &num_corners);
            scores = fast9_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            if (nonmaxSuppression) {
                int num_nonmax_corners;
                xy *nonmax = nonmax_suppression(corners, scores, num_corners, &num_nonmax_corners);
                free(corners);
                free(scores);
                corners = nonmax;
                num_corners = num_nonmax_corners;
                scores = fast9_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            }
            break;
        case FAST_10:
            corners = fast10_detect((const byte*)frame.data(), frame.getWidth(), frame.getHeight(), frame.getWidth(), threshold, &num_corners);
            scores = fast10_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            if (nonmaxSuppression) {
                int num_nonmax_corners;
                xy *nonmax = nonmax_suppression(corners, scores, num_corners, &num_nonmax_corners);
                free(corners);
                free(scores);
                corners = nonmax;
                num_corners = num_nonmax_corners;
                scores = fast10_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            }
            break;
        case FAST_11:
            corners = fast11_detect((const byte*)frame.data(), frame.getWidth(), frame.getHeight(), frame.getWidth(), threshold, &num_corners);
            scores = fast11_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            if (nonmaxSuppression) {
                int num_nonmax_corners;
                xy *nonmax = nonmax_suppression(corners, scores, num_corners, &num_nonmax_corners);
                free(corners);
                free(scores);
                corners = nonmax;
                num_corners = num_nonmax_corners;
                scores = fast11_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            }
            break;
        case FAST_12:
            corners = fast12_detect((const byte*)frame.data(), frame.getWidth(), frame.getHeight(), frame.getWidth(), threshold, &num_corners);
            scores = fast12_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            if (nonmaxSuppression) {
                int num_nonmax_corners;
                xy *nonmax = nonmax_suppression(corners, scores, num_corners, &num_nonmax_corners);
                free(corners);
                free(scores);
                corners = nonmax;
                num_corners = num_nonmax_corners;
                scores = fast12_score((const byte*)frame.data(), frame.getWidth(), corners, num_corners, threshold);
            }
            break;
        default:
            assertThrow(false, "Unknown FAST type");
    }


    points.reserve(num_corners);
    for (int i = 0; i < num_corners; i++) {
        Corner corner(DistortedVector2d(corners[i].x, corners[i].y));
        corner.setResponse(scores[i]);
        points.emplace_back(corner);
    }

    free(corners);
    free(scores);

}