#include "cml/features/corner/OpenCV.h"

void
CML::Features::OpenCV::ORB::compute(const CaptureImage &frame, List<Corner> &corners, List<Descriptor> &descriptors) {
    GrayImage cmlImage = frame.getGrayImage(0).cast<unsigned char>();
    cv::Mat image(cv::Size(frame.getWidth(0), frame.getHeight(0)), CV_8UC1, (void *) cmlImage.data(),
                  cv::Mat::AUTO_STEP);

    corners.reserve(mNumFeatures);
    descriptors.reserve(mNumFeatures);

    cv::Mat featuresMatrix;
    std::vector<cv::KeyPoint> kpts;

    auto orb = ::cv::ORB::create(mNumFeatures);
    orb->detectAndCompute(image, cv::Mat(), kpts, featuresMatrix);


    if ((int) kpts.size() > 0) {

        std::vector<cv::Point2f> points;
        points.resize(kpts.size());
        for (size_t i = 0; i < kpts.size(); i++) {
            points[i] = kpts[i].pt;
        }

        cv::Size winSize = cv::Size(5, 5);
        cv::Size zeroZone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001);
        cv::cornerSubPix(image, points, winSize, zeroZone, criteria);

        for (size_t i = 0; i < kpts.size(); i++) {
            Corner corner = Corner(DistortedVector2d(points[i].x + 0.5, points[i].y + 0.5));
            corner.setAngle(kpts[i].angle);
            corner.setLevel(kpts[i].octave);
            corner.setResponse(kpts[i].response);
            corner.setScaleFactor(1.2f);
            corners.emplace_back(corner);
        }

        for (size_t i = 0; i < kpts.size(); i++) {
            unsigned char descriptor[32];
            for (unsigned int j = 0; j < 32; j++) {
                descriptor[j] = featuresMatrix.at<unsigned char>(i, j);
            }
            descriptors.emplace_back(Binary256Descriptor(descriptor));
        }

    }


    mLastCorners = corners;


}

int CML::Features::OpenCV::ORB::compute(PFrame frame, List<Descriptor> &descriptors) {
    List<Corner> corners;
    int group;

    compute(frame->getCaptureFrame(), corners, descriptors);
    if (corners.size() == 0) {
        return -1;
    }
    group = frame->addFeaturePoints(corners);

    return group;
}

void CML::Features::OpenCV::ORB::compute(const CaptureImage &frame, List<Corner> &corners,
                                         List<Binary256Descriptor> &descriptors, Ptr<BoW, Nullable> &bow) {
    compute(frame, corners, descriptors);
    if (bow.isNull()) {
        bow = new BoW();
    }
    mVocabulary->transform(descriptors, bow->bowVec, bow->featVec, 4);
}
