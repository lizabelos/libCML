#ifndef CML_BOWTRACKER_H
#define CML_BOWTRACKER_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/bow/Bow.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

namespace CML::Features {

    inline bool isInFrustum(PPoint point, PFrame frame) {
        if (!frame->getCamera().inFront(point->getWorldCoordinate().absolute())) {
            return false;
        }

        DistortedVector2d projection = frame->distort(point->getWorldCoordinate().project(frame->getCamera()), 0);

        if (!frame->isInside(projection, 0, 0)) {
            return false;
        }

        scalar_t scaleFactor = point->getReferenceCorner().processScaleFactorFromLevel();
        scalar_t maxScaleFactor =  point->getReferenceCorner().processScaleFactorFromLevel(7); // todo : 7 is 8 - 1

        // Check distance is in the scale invariance region of the MapPoint
        Vector3 PO = point->getWorldCoordinate().absolute() - frame->getCamera().eye();
        Vector3 Pn = point->getWorldCoordinate().absolute() - point->getReferenceFrame()->getCamera().eye();
        scalar_t POdist = PO.norm();
        scalar_t Pndist = Pn.norm();
        scalar_t maxDistance = Pndist * scaleFactor;
        scalar_t minDistance = maxDistance / maxScaleFactor;

        if(POdist<0.8*minDistance || POdist>1.2*maxDistance) {
            return false;
        }

        // Check viewing angle

        scalar_t viewCos = PO.dot(Pn) / POdist;

        if(viewCos < 0.5) {
            return false;
        }

        return true;
    }

    inline bool computeViewcosAndScale(PPoint point, PFrame frame, const Camera &camera, DistortedVector2d &projection, scalar_t &viewCos, int &predictedScale) {

        if (!camera.inFront(point->getWorldCoordinate().absolute())) {
            return false;
        }

        projection = frame->distort(point->getWorldCoordinate().project(camera), 0);

        if (!frame->isInside(projection, 0, 0)) {
            return false;
        }

        scalar_t scaleFactor = point->getReferenceCorner().processScaleFactorFromLevel();
        scalar_t maxScaleFactor =  point->getReferenceCorner().processScaleFactorFromLevel(7); // todo : 7 is 8 - 1

        // Check distance is in the scale invariance region of the MapPoint
        Vector3 PO = point->getWorldCoordinate().absolute() - camera.eye();
        Vector3 Pn = point->getWorldCoordinate().absolute() - point->getReferenceFrame()->getCamera().eye();
        scalar_t POdist = PO.norm();
        scalar_t Pndist = Pn.norm();
        scalar_t maxDistance = Pndist * scaleFactor;
        scalar_t minDistance = maxDistance / maxScaleFactor;

        if(POdist<0.8*minDistance || POdist>1.2*maxDistance) {
            return false;
        }

        // Check viewing angle

        viewCos = PO.dot(Pn) / POdist;

        if(viewCos < 0.5) {
            return false;
        }

        // Predict scale in the image

        scalar_t ratio = maxDistance / POdist;

        predictedScale = ceil(log(ratio) / log(scaleFactor));
        if(predictedScale < 0) {
            predictedScale = 0;
        }
        else if(predictedScale >= 8) { // todo : 8 is the orb level, need to propagate the info here. put it in the feautre point ?
            predictedScale = 7; // todo : 7 is 8 - 1
        }
        return true;

    }

    inline void ComputeThreeMaxima(CML::List<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }

    typedef struct BoWFrameAndGroupAndDescriptor {

        BoWFrameAndGroupAndDescriptor(PFrame _frame, int _group, const List<Binary256Descriptor> &_descriptors) :
        frame(_frame), group(_group), descriptors(_descriptors)
        {
            camera = frame->getCamera();
        }

        BoWFrameAndGroupAndDescriptor(PFrame _frame, int _group, const List<Binary256Descriptor> &_descriptors, Camera _camera) :
                frame(_frame), group(_group), descriptors(_descriptors), camera(_camera)
        {

        }

        PFrame frame;
        int group;
        const List<Binary256Descriptor> &descriptors;
        Camera camera;
    } BoWFrameAndGroupAndDescriptor;

    typedef struct BoWFrameAndGroup {

        BoWFrameAndGroup(PFrame _frame, int _group) :
        frame(_frame), group(_group)
        {
            camera = frame->getCamera();
        }

        BoWFrameAndGroup(PFrame _frame, int _group, Camera _camera) :
        frame(_frame), group(_group), camera(_camera)
        {

        }

        PFrame frame;
        int group;
        Camera camera;
    } BoWFrameAndGroup;

    class BoWTracker : public AbstractFunction {

    public:
        BoWTracker(Ptr<AbstractFunction, Nullable> parent, float ratio, bool checkOrientation) : AbstractFunction(parent) {
            mRatio.set(ratio);
            mCheckOrientation.set(checkOrientation);
            mUnfilteredNearestNeighbor.reserve(1024);
            mNearestNeighbor.reserve(1024);
        }

        List<Pair<int, int>> trackByProjection(const BoWFrameAndGroupAndDescriptor &A, const List<PPoint> &mapPoints, int th = 1);

        List<Matching> trackByBoW(const BoWFrameAndGroupAndDescriptor &A, const BoWFrameAndGroupAndDescriptor &B);

        List<Matching> trackForInitialization(const BoWFrameAndGroupAndDescriptor &target, const BoWFrameAndGroupAndDescriptor &reference, int windowSize);

        List<Matching> trackForTriangulation(const BoWFrameAndGroupAndDescriptor &A, const BoWFrameAndGroupAndDescriptor &B, int th = 1);

        List<Matching> trackByProjection(const BoWFrameAndGroupAndDescriptor &A, const BoWFrameAndGroup &B, int th = 1);

        PointSet fuse(const BoWFrameAndGroupAndDescriptor &A, const List<PPoint> &mapPoints);

        std::string getName() override {
            return "BoWTracker";
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) override {
            List<Matching> lastMatchings;
            {
                LockGuard lg(mLastMatchingMutex);
                lastMatchings = mLastMatching;
            }

            for (const auto &matching : lastMatchings) {
                drawBoard.lineWidth(1);
                drawBoard.color(0, 1, 0);
                drawBoard.segment(
                        (Eigen::Vector2f)matching.noAssertGetFeaturePointA().point(0).cast<float>(),
                        (Eigen::Vector2f)matching.noAssertGetFeaturePointB().point(0).cast<float>()
                        );

                drawBoard.pointSize(5);
                drawBoard.color(1, 0, 0);
                drawBoard.point((Eigen::Vector2f)matching.noAssertGetFeaturePointA().point(0).cast<float>());
                drawBoard.pointSize(3);
                drawBoard.color(0, 0, 0);
                drawBoard.point((Eigen::Vector2f)matching.noAssertGetFeaturePointA().point(0).cast<float>());
            }

        }

        void viewOnReconstruction(DrawBoard &drawBoard) override {
            List<Matching> lastMatchings;
            {
                LockGuard lg(mLastMatchingMutex);
                lastMatchings = mLastMatching;
            }


            for (const auto &matching : lastMatchings) {
                drawBoard.pointSize(5);
                drawBoard.color(1, 0, 0);
                if (matching.getMapPoint().isNotNull()) {
                    drawBoard.point((Eigen::Vector3f)matching.getMapPoint()->getWorldCoordinate().absolute().cast<float>());
                }
            }
        }


    private:
        Parameter mRatio = createParameter("ratio", 0.6f);
        Parameter mCheckOrientation = createParameter("checkOrientation", true);

        Parameter mThHigh = createParameter("thHigh", 100);
        Parameter mThLow = createParameter("thLow", 50);
        Parameter mHistoLenght = createParameter("histoLength", 30);

        Mutex mLastMatchingMutex;
        List<Matching> mLastMatching;

        PStatistic mNumMatching = createStatistic("Matching Number");
        PStatistic mNumMatchingMapped = createStatistic("Mapped Matching number");

        List<Corner> mInitializationLastSeen;
        OptPFrame mInitializationLastReference;

        List<NearestNeighbor> mUnfilteredNearestNeighbor, mNearestNeighbor;

    };


}

#endif