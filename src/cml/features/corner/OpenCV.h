#ifndef CML_FEATURES_CORNER_OPENCV
#define CML_FEATURES_CORNER_OPENCV

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <cml/features/Features.h>

namespace CML::Features::OpenCV {

    inline cv::Mat toCvMat(const GrayImage &image) {
        return cv::Mat(cv::Size(image.getWidth(), image.getHeight()), CV_8UC1, (void*)image.data(), cv::Mat::AUTO_STEP);
    }

    inline GrayImage toGrayImage(const cv::Mat &mat) {
        return GrayImage(mat.cols, mat.rows, mat.data);
    }

    inline Corner toCorner(const cv::KeyPoint &kp) {
        Corner corner = Corner(DistortedVector2d(kp.pt.x + 0.5, kp.pt.y + 0.5));
        corner.setAngle(kp.angle);
        corner.setLevel(kp.octave);
        corner.setResponse(kp.response);
        corner.setScaleFactor(1.0);
        return corner;
    }

    inline List<Corner> toCorners(const std::vector<cv::KeyPoint> &kp) {
        List<Corner> corners(kp.size());
        for (size_t i = 0; i < kp.size(); i++) {
            corners[i] = toCorner(kp[i]);
        }
        return corners;
    }

    inline GrayImage gaussianBlur(const GrayImage &image, int x, int y, int sx, int sy) {
        cv::Mat mat = toCvMat(image);
        cv::GaussianBlur(mat, mat, cv::Size(x, y), sx, sy, cv::BORDER_REFLECT_101);
        return toGrayImage(mat);
    }

    inline GrayImage medianBlur(const GrayImage &image, int x) {
        cv::Mat mat = toCvMat(image);
        cv::medianBlur(mat, mat, x);
        return toGrayImage(mat);
    }

    inline GrayImage blur(const GrayImage &image, int x, int y) {
        cv::Mat mat = toCvMat(image);
        cv::blur(mat, mat, cv::Size(x, y), cv::Point(-1, -1), cv::BORDER_REFLECT_101);
        return toGrayImage(mat);
    }

    inline GrayImage resize(const GrayImage &image, int w, int h) {
        cv::Mat input = toCvMat(image), output;
        cv::resize(input, output, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
        return toGrayImage(output);
    }

    inline List<Corner> FAST(const GrayImage &image, int threshold, bool nonmaxSuppression = true) {
        std::vector<cv::KeyPoint> kps;
        ::cv::FAST(toCvMat(image), kps, threshold, nonmaxSuppression);
        return toCorners(kps);
    }

    typedef TemplatedVocabulary<Binary256Descriptor> ORBVocabulary;

    class ORB : public AbstractFunction {

    public:
        using Descriptor = Binary256Descriptor;

        ORB(Ptr<AbstractFunction, NonNullable> parent, int numFeatures) : AbstractFunction(parent) {
            mNumFeatures = numFeatures;
        }

        inline std::string getName() final {
            return "OpenCV ORB";
        }

        void loadVocabulary(const std::string &filename) {
            mVocabulary = new ORBVocabulary();
            mVocabulary->loadFromTextFile(filename);
            if (mVocabulary->empty()) {
                throw std::runtime_error("Can't load vocabulary");
            }
        }

        const ORBVocabulary& getVocabulary() {
            return *mVocabulary.p();
        }

        void compute(const CaptureImage &frame, List<Corner> &corners, List <Descriptor> &descriptors);

        int compute(PFrame frame, List<Descriptor> &descriptors);

        void compute(const CaptureImage &frame, List<Corner> &corners, List<Binary256Descriptor> &descriptors, Ptr<BoW, Nullable> &bow);

        ORB *setNumFeatures(int n) {
            mNumFeatures = n;
            return this;
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final {
            List<Corner> lastCorners;
            {
                LockGuard lg(mLastCornersMutex);
                lastCorners = mLastCorners;
            }
            drawBoard.color(1, 0, 1);
            drawBoard.pointSize(1);
            for (auto corner : lastCorners) {
                drawBoard.point((Vector2f)corner.point0().cast<float>());
            }
        }

    private:
        int mNumFeatures;
        Ptr<ORBVocabulary, Nullable> mVocabulary;

        Mutex mLastCornersMutex;
        List<Corner> mLastCorners;

    };

}

#endif