#include "cml/features/corner/BRIEF.h"
#include "BRIEFPattern.cpp"

void CML::Features::BRIEF::compute(const CaptureImage &frame, List<Corner> &corners, List<Binary256Descriptor> &descriptors) {
#if CML_CAPTUREIMAGE_REDUCEMEMORYUSAGE
    auto image = frame.getGrayImage(0).children();
#else
    auto image = frame.getGrayImage(0).cast<unsigned char>();
#endif
    mFast.compute(image, corners, mThreshold.i());
    descriptors.resize(corners.size());
    for (size_t i = 0; i < corners.size(); i++) {
        descriptors[i] = computeDescriptor(image, corners[i]);
    }
}

void CML::Features::BRIEF::compute(const CaptureImage &frame, List<Corner> &corners, List<Binary256Descriptor> &descriptors, Ptr<Features::BoW, Nullable> &bow) {
    compute(frame, corners, descriptors);
    if (bow.isNull()) {
        bow = new BoW();
    }
    mVocabulary->transform(descriptors,bow->bowVec,bow->featVec,4);
}

CML::Binary256Descriptor CML::Features::BRIEF::computeDescriptor(const Array2D<unsigned char> &frame, Corner &corner) {

    #define GET_VALUE(i, j) (*(image_center + ia_y[i*32 + j]*stride_image + ia_x[i*32 + j]) < *(image_center + ib_y[i*32 + j]*stride_image + ib_x[i*32 + j])) << j

    unsigned char *image_src = (unsigned char*)frame.data();
    DistortedVector2d point = corner.point(0);

    int stride_image = frame.getWidth();

    if (
            (point.x() <= diag_length_pattern) || point.x() >= (frame.getWidth() - diag_length_pattern) ||
            (point.y() <= diag_length_pattern) || point.y() >= (frame.getHeight() - diag_length_pattern)
            ) {
       // CML_LOG_ERROR("Can't compute BRIEF descriptor : padding are not respected");
        return Binary256Descriptor();
    }

    float cos_angle = cos(corner.angle());
    float sin_angle = sin(corner.angle());
    unsigned char *image_center = image_src + (int) point.y() * stride_image + (int) point.x();
    // N_DIM_BINARYDESCRIPTOR / SIZE_BITS_HAMING = 4
    alignas(64) int32_t ia_x[256];
    alignas(64) int32_t ia_y[256];
    alignas(64) int32_t ib_x[256];
    alignas(64) int32_t ib_y[256];

    for (int i = 0; i < 256; i++) {
        ia_x[i] = round((gaussian_bit_pattern_31_x_a[i] * cos_angle - gaussian_bit_pattern_31_y_a[i] * sin_angle));
        ia_y[i] = round((gaussian_bit_pattern_31_x_a[i] * sin_angle + gaussian_bit_pattern_31_y_a[i] * cos_angle));
        ib_x[i] = round((gaussian_bit_pattern_31_x_b[i] * cos_angle - gaussian_bit_pattern_31_y_b[i] * sin_angle));
        ib_y[i] = round((gaussian_bit_pattern_31_x_b[i] * sin_angle + gaussian_bit_pattern_31_y_b[i] * cos_angle));
    }


    alignas(64) int32_t f[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    f[0] |= GET_VALUE(0, 0);
    f[0] |= GET_VALUE(0, 1);
    f[0] |= GET_VALUE(0, 2);
    f[0] |= GET_VALUE(0, 3);
    f[0] |= GET_VALUE(0, 4);
    f[0] |= GET_VALUE(0, 5);
    f[0] |= GET_VALUE(0, 6);
    f[0] |= GET_VALUE(0, 7);
    f[0] |= GET_VALUE(0, 8);
    f[0] |= GET_VALUE(0, 9);
    f[0] |= GET_VALUE(0, 10);
    f[0] |= GET_VALUE(0, 11);
    f[0] |= GET_VALUE(0, 12);
    f[0] |= GET_VALUE(0, 13);
    f[0] |= GET_VALUE(0, 14);
    f[0] |= GET_VALUE(0, 15);
    f[0] |= GET_VALUE(0, 16);
    f[0] |= GET_VALUE(0, 17);
    f[0] |= GET_VALUE(0, 18);
    f[0] |= GET_VALUE(0, 19);
    f[0] |= GET_VALUE(0, 20);
    f[0] |= GET_VALUE(0, 21);
    f[0] |= GET_VALUE(0, 22);
    f[0] |= GET_VALUE(0, 23);
    f[0] |= GET_VALUE(0, 24);
    f[0] |= GET_VALUE(0, 25);
    f[0] |= GET_VALUE(0, 26);
    f[0] |= GET_VALUE(0, 27);
    f[0] |= GET_VALUE(0, 28);
    f[0] |= GET_VALUE(0, 29);
    f[0] |= GET_VALUE(0, 30);
    f[0] |= GET_VALUE(0, 31);

    f[1] |= GET_VALUE(1, 0);
    f[1] |= GET_VALUE(1, 1);
    f[1] |= GET_VALUE(1, 2);
    f[1] |= GET_VALUE(1, 3);
    f[1] |= GET_VALUE(1, 4);
    f[1] |= GET_VALUE(1, 5);
    f[1] |= GET_VALUE(1, 6);
    f[1] |= GET_VALUE(1, 7);
    f[1] |= GET_VALUE(1, 8);
    f[1] |= GET_VALUE(1, 9);
    f[1] |= GET_VALUE(1, 10);
    f[1] |= GET_VALUE(1, 11);
    f[1] |= GET_VALUE(1, 12);
    f[1] |= GET_VALUE(1, 13);
    f[1] |= GET_VALUE(1, 14);
    f[1] |= GET_VALUE(1, 15);
    f[1] |= GET_VALUE(1, 16);
    f[1] |= GET_VALUE(1, 17);
    f[1] |= GET_VALUE(1, 18);
    f[1] |= GET_VALUE(1, 19);
    f[1] |= GET_VALUE(1, 20);
    f[1] |= GET_VALUE(1, 21);
    f[1] |= GET_VALUE(1, 22);
    f[1] |= GET_VALUE(1, 23);
    f[1] |= GET_VALUE(1, 24);
    f[1] |= GET_VALUE(1, 25);
    f[1] |= GET_VALUE(1, 26);
    f[1] |= GET_VALUE(1, 27);
    f[1] |= GET_VALUE(1, 28);
    f[1] |= GET_VALUE(1, 29);
    f[1] |= GET_VALUE(1, 30);
    f[1] |= GET_VALUE(1, 31);

    f[2] |= GET_VALUE(2, 0);
    f[2] |= GET_VALUE(2, 1);
    f[2] |= GET_VALUE(2, 2);
    f[2] |= GET_VALUE(2, 3);
    f[2] |= GET_VALUE(2, 4);
    f[2] |= GET_VALUE(2, 5);
    f[2] |= GET_VALUE(2, 6);
    f[2] |= GET_VALUE(2, 7);
    f[2] |= GET_VALUE(2, 8);
    f[2] |= GET_VALUE(2, 9);
    f[2] |= GET_VALUE(2, 10);
    f[2] |= GET_VALUE(2, 11);
    f[2] |= GET_VALUE(2, 12);
    f[2] |= GET_VALUE(2, 13);
    f[2] |= GET_VALUE(2, 14);
    f[2] |= GET_VALUE(2, 15);
    f[2] |= GET_VALUE(2, 16);
    f[2] |= GET_VALUE(2, 17);
    f[2] |= GET_VALUE(2, 18);
    f[2] |= GET_VALUE(2, 19);
    f[2] |= GET_VALUE(2, 20);
    f[2] |= GET_VALUE(2, 21);
    f[2] |= GET_VALUE(2, 22);
    f[2] |= GET_VALUE(2, 23);
    f[2] |= GET_VALUE(2, 24);
    f[2] |= GET_VALUE(2, 25);
    f[2] |= GET_VALUE(2, 26);
    f[2] |= GET_VALUE(2, 27);
    f[2] |= GET_VALUE(2, 28);
    f[2] |= GET_VALUE(2, 29);
    f[2] |= GET_VALUE(2, 30);
    f[2] |= GET_VALUE(2, 31);

    f[3] |= GET_VALUE(3, 0);
    f[3] |= GET_VALUE(3, 1);
    f[3] |= GET_VALUE(3, 2);
    f[3] |= GET_VALUE(3, 3);
    f[3] |= GET_VALUE(3, 4);
    f[3] |= GET_VALUE(3, 5);
    f[3] |= GET_VALUE(3, 6);
    f[3] |= GET_VALUE(3, 7);
    f[3] |= GET_VALUE(3, 8);
    f[3] |= GET_VALUE(3, 9);
    f[3] |= GET_VALUE(3, 10);
    f[3] |= GET_VALUE(3, 11);
    f[3] |= GET_VALUE(3, 12);
    f[3] |= GET_VALUE(3, 13);
    f[3] |= GET_VALUE(3, 14);
    f[3] |= GET_VALUE(3, 15);
    f[3] |= GET_VALUE(3, 16);
    f[3] |= GET_VALUE(3, 17);
    f[3] |= GET_VALUE(3, 18);
    f[3] |= GET_VALUE(3, 19);
    f[3] |= GET_VALUE(3, 20);
    f[3] |= GET_VALUE(3, 21);
    f[3] |= GET_VALUE(3, 22);
    f[3] |= GET_VALUE(3, 23);
    f[3] |= GET_VALUE(3, 24);
    f[3] |= GET_VALUE(3, 25);
    f[3] |= GET_VALUE(3, 26);
    f[3] |= GET_VALUE(3, 27);
    f[3] |= GET_VALUE(3, 28);
    f[3] |= GET_VALUE(3, 29);
    f[3] |= GET_VALUE(3, 30);
    f[3] |= GET_VALUE(3, 31);

    f[4] |= GET_VALUE(4, 0);
    f[4] |= GET_VALUE(4, 1);
    f[4] |= GET_VALUE(4, 2);
    f[4] |= GET_VALUE(4, 3);
    f[4] |= GET_VALUE(4, 4);
    f[4] |= GET_VALUE(4, 5);
    f[4] |= GET_VALUE(4, 6);
    f[4] |= GET_VALUE(4, 7);
    f[4] |= GET_VALUE(4, 8);
    f[4] |= GET_VALUE(4, 9);
    f[4] |= GET_VALUE(4, 10);
    f[4] |= GET_VALUE(4, 11);
    f[4] |= GET_VALUE(4, 12);
    f[4] |= GET_VALUE(4, 13);
    f[4] |= GET_VALUE(4, 14);
    f[4] |= GET_VALUE(4, 15);
    f[4] |= GET_VALUE(4, 16);
    f[4] |= GET_VALUE(4, 17);
    f[4] |= GET_VALUE(4, 18);
    f[4] |= GET_VALUE(4, 19);
    f[4] |= GET_VALUE(4, 20);
    f[4] |= GET_VALUE(4, 21);
    f[4] |= GET_VALUE(4, 22);
    f[4] |= GET_VALUE(4, 23);
    f[4] |= GET_VALUE(4, 24);
    f[4] |= GET_VALUE(4, 25);
    f[4] |= GET_VALUE(4, 26);
    f[4] |= GET_VALUE(4, 27);
    f[4] |= GET_VALUE(4, 28);
    f[4] |= GET_VALUE(4, 29);
    f[4] |= GET_VALUE(4, 30);
    f[4] |= GET_VALUE(4, 31);

    f[5] |= GET_VALUE(5, 0);
    f[5] |= GET_VALUE(5, 1);
    f[5] |= GET_VALUE(5, 2);
    f[5] |= GET_VALUE(5, 3);
    f[5] |= GET_VALUE(5, 4);
    f[5] |= GET_VALUE(5, 5);
    f[5] |= GET_VALUE(5, 6);
    f[5] |= GET_VALUE(5, 7);
    f[5] |= GET_VALUE(5, 8);
    f[5] |= GET_VALUE(5, 9);
    f[5] |= GET_VALUE(5, 10);
    f[5] |= GET_VALUE(5, 11);
    f[5] |= GET_VALUE(5, 12);
    f[5] |= GET_VALUE(5, 13);
    f[5] |= GET_VALUE(5, 14);
    f[5] |= GET_VALUE(5, 15);
    f[5] |= GET_VALUE(5, 16);
    f[5] |= GET_VALUE(5, 17);
    f[5] |= GET_VALUE(5, 18);
    f[5] |= GET_VALUE(5, 19);
    f[5] |= GET_VALUE(5, 20);
    f[5] |= GET_VALUE(5, 21);
    f[5] |= GET_VALUE(5, 22);
    f[5] |= GET_VALUE(5, 23);
    f[5] |= GET_VALUE(5, 24);
    f[5] |= GET_VALUE(5, 25);
    f[5] |= GET_VALUE(5, 26);
    f[5] |= GET_VALUE(5, 27);
    f[5] |= GET_VALUE(5, 28);
    f[5] |= GET_VALUE(5, 29);
    f[5] |= GET_VALUE(5, 30);
    f[5] |= GET_VALUE(5, 31);

    f[6] |= GET_VALUE(6, 0);
    f[6] |= GET_VALUE(6, 1);
    f[6] |= GET_VALUE(6, 2);
    f[6] |= GET_VALUE(6, 3);
    f[6] |= GET_VALUE(6, 4);
    f[6] |= GET_VALUE(6, 5);
    f[6] |= GET_VALUE(6, 6);
    f[6] |= GET_VALUE(6, 7);
    f[6] |= GET_VALUE(6, 8);
    f[6] |= GET_VALUE(6, 9);
    f[6] |= GET_VALUE(6, 10);
    f[6] |= GET_VALUE(6, 11);
    f[6] |= GET_VALUE(6, 12);
    f[6] |= GET_VALUE(6, 13);
    f[6] |= GET_VALUE(6, 14);
    f[6] |= GET_VALUE(6, 15);
    f[6] |= GET_VALUE(6, 16);
    f[6] |= GET_VALUE(6, 17);
    f[6] |= GET_VALUE(6, 18);
    f[6] |= GET_VALUE(6, 19);
    f[6] |= GET_VALUE(6, 20);
    f[6] |= GET_VALUE(6, 21);
    f[6] |= GET_VALUE(6, 22);
    f[6] |= GET_VALUE(6, 23);
    f[6] |= GET_VALUE(6, 24);
    f[6] |= GET_VALUE(6, 25);
    f[6] |= GET_VALUE(6, 26);
    f[6] |= GET_VALUE(6, 27);
    f[6] |= GET_VALUE(6, 28);
    f[6] |= GET_VALUE(6, 29);
    f[6] |= GET_VALUE(6, 30);
    f[6] |= GET_VALUE(6, 31);

    f[7] |= GET_VALUE(7, 0);
    f[7] |= GET_VALUE(7, 1);
    f[7] |= GET_VALUE(7, 2);
    f[7] |= GET_VALUE(7, 3);
    f[7] |= GET_VALUE(7, 4);
    f[7] |= GET_VALUE(7, 5);
    f[7] |= GET_VALUE(7, 6);
    f[7] |= GET_VALUE(7, 7);
    f[7] |= GET_VALUE(7, 8);
    f[7] |= GET_VALUE(7, 9);
    f[7] |= GET_VALUE(7, 10);
    f[7] |= GET_VALUE(7, 11);
    f[7] |= GET_VALUE(7, 12);
    f[7] |= GET_VALUE(7, 13);
    f[7] |= GET_VALUE(7, 14);
    f[7] |= GET_VALUE(7, 15);
    f[7] |= GET_VALUE(7, 16);
    f[7] |= GET_VALUE(7, 17);
    f[7] |= GET_VALUE(7, 18);
    f[7] |= GET_VALUE(7, 19);
    f[7] |= GET_VALUE(7, 20);
    f[7] |= GET_VALUE(7, 21);
    f[7] |= GET_VALUE(7, 22);
    f[7] |= GET_VALUE(7, 23);
    f[7] |= GET_VALUE(7, 24);
    f[7] |= GET_VALUE(7, 25);
    f[7] |= GET_VALUE(7, 26);
    f[7] |= GET_VALUE(7, 27);
    f[7] |= GET_VALUE(7, 28);
    f[7] |= GET_VALUE(7, 29);
    f[7] |= GET_VALUE(7, 30);
    f[7] |= GET_VALUE(7, 31);

    return Binary256Descriptor((uint64_t*)f);

#undef ROUND
#undef GET_VALUE

}

int CML::Features::BRIEF::compute(PFrame frame, List<Binary256Descriptor> &descriptors) {

    List<Corner> corners;
    int group;
    if (mVocabulary.isNotNull()) {
        Ptr<BoW, Nullable> bow;
        compute(frame->getCaptureFrame(), corners, descriptors, bow);
        if (corners.size() == 0) {
            return -1;
        }
        group = frame->addFeaturePoints(corners, bow);
    } else {
        compute(frame->getCaptureFrame(), corners, descriptors);
        if (corners.size() == 0) {
            return -1;
        }
        group = frame->addFeaturePoints(corners);
    }
    return group;
}
