//
// Created by tbelos on 19/04/19.
//

#ifndef CML_MAP_H
#define CML_MAP_H

#include <cml/config.h>
#include <cml/map/MapObject.h>
#include <cml/map/Frame.h>
#include <cml/features/Features.h>
#include <cml/utils/GarbageCollector.h>
#include <cml/map/GroupsManager.h>

#include <memory>
#include <vector>
#include <thread>
#include <mutex>

namespace CML {

#define CML_MAP_MAPPOINT_BUFFER_NUM 8192
#define CML_MAP_MAPPOINT_BUFFER_SIZE (CML_MAPPOINT_SPARSITY * CML_MAP_MAPPOINT_BUFFER_NUM)
#define CML_MAP_FRAME_BUFFER_SIZE 40696

    typedef enum MapResultFormat {
        MAP_RESULT_FORMAT_TUM, MAP_RESULT_FORMAT_KITTI
    } MapResultFormat;

    class Map : private Frame::Observer, private MapPoint::Observer {

        friend class Frame;
        friend class MapPoint;

    public:
        Map();

        ~Map();

        class Observer : public DeterministicallyHashable {

        public:
            virtual void onAddFrame(Map &map, PFrame frame) {

            }

            virtual void onFrameChangeGroup(Map &map, PFrame frame, int groupId, bool state) {

            }

            virtual void onMapPointChangeGroup(Map &map, PPoint mapPoint, int groupId, bool state) {

            }

        };

        inline void subscribeObserver(Observer *observer) {
            assertThrow(observer != nullptr, "null observer passed to subscribeObserver");
            LockGuard lg(mObserversMutex);
            mObservers.insert(observer);
        }

        inline void removeObserver(Observer *observer) {
            LockGuard lg(mObserversMutex);
            mObservers.erase(observer);
        }


        PPoint createMapPoint(PFrame reference, FeatureIndex referenceIndex, MapPointType type);

        PFrame createFrame(Ptr<CaptureImage, NonNullable> captureFrame);

        void removeMapPoint(PPoint mapPoint, bool singleHolder = false);

        void addFrame(PFrame frame);

        Set<PPoint> getMapPoints();

        int getMapPointsNumber();

        Set<PPoint> getGroupMapPoints(int groupId);

        inline List<PPoint> getGroupMapPointsAsList(int groupId) {
            auto points = getGroupMapPoints(groupId);
            return List<PPoint>(points.begin(), points.end());
        }

        OrderedSet<PFrame, Comparator> getFrames();

        inline List<PFrame> getFrameAsList() {
            auto frames = getFrames();
            return List<PFrame>(frames.begin(), frames.end());
        }

        PFrame getLastFrame();

        PFrame getLastFrame(int n);

        unsigned int getFramesNumber();

        unsigned int getGroupFramesNumber(int groupId);

        inline List<PFrame> getGroupFrameAsList(int groupId) {
            auto frames = getGroupFrames(groupId);
            return List<PFrame>(frames.begin(), frames.end());
        }

        OptPFrame getLastGroupFrame(int groupId);

        OptPFrame getLastGroupFrame(int groupId, int n);

        OrderedSet<PFrame, Comparator> getGroupFrames(int groupId);

        void reset();

        List<PFrame> processIndirectCovisiblity(PFrame frame, int max = -1, int groupId = -1, int th = -1);

        List<PFrame> processDirectCovisiblity(PFrame frame, int max = -1, int groupId = -1);


        GarbageCollector &getGarbageCollector() {
            return mGarbageCollector;
        }

        void refreshErrorFromGroundtruth();

        EIGEN_STRONG_INLINE List<scalar_t> getATE() {
            LockGuard lg(mErrorMutex);
            return mATE;
        }

        EIGEN_STRONG_INLINE List<Camera> getAlignedGroudtruth() {
            LockGuard lg(mErrorMutex);
            return mAlignedGronudtruth;
        }

        EIGEN_STRONG_INLINE List<scalar_t> getRPE() {
            LockGuard lg(mErrorMutex);
            return mRPE;
        }

        EIGEN_STRONG_INLINE List<scalar_t*> &getBufferCoordinates() {
            return mBufferCoordinates;
        }

        EIGEN_STRONG_INLINE List<scalar_t*> &getBufferColors() {
            return mBufferColors;
        }

        EIGEN_STRONG_INLINE List<scalar_t*> &getUncertainties() {
            return mBufferUncertainty;
        }

        EIGEN_STRONG_INLINE List<unsigned int*> &getBufferGroups() {
            return mBufferGroups;
        }

        EIGEN_STRONG_INLINE List<scalar_t*> &getCameraCenters() {
            return mBufferFrameCenter;
        }

        EIGEN_STRONG_INLINE List<int> &getCameraCentersSize() {
            return mBufferFrameCenterSize;
        }

        EIGEN_STRONG_INLINE int createFrameGroup(std::string name) {
            return mFrameGroupsManager.createGroup(name);
        }

        EIGEN_STRONG_INLINE int createMapPointGroup(std::string name) {
            return mMapPointsGroupManager.createGroup(name);
        }

        EIGEN_STRONG_INLINE const GroupsManager &getFrameGroupsManager() {
            return mFrameGroupsManager;
        }

        EIGEN_STRONG_INLINE const GroupsManager &getMapPointsGroupsManager() {
            return mMapPointsGroupManager;
        }

        EIGEN_STRONG_INLINE PrivateDataContext &getFramePrivataDataContext() {
            return mFramePrivateDataContext;
        }

        EIGEN_STRONG_INLINE PrivateDataContext &getMapPointsPrivataDataContext() {
            return mMapPointsPrivateDataContext;
        }

        void exportResults(std::string path, MapResultFormat format, bool exportGroundtruth = false);

        bool canMargePoints(PPoint mapPointA, PPoint mapPointB);

        bool mergeMapPoints(PPoint mapPointA, PPoint mapPointB);

        List<Camera> multiConstantVelocityMotionModel(PFrame frame, int nFrames = 10) {

            List<Camera> results;

            List<PFrame> frames = getFrameAsList();

            int i = 0;
            for (; i < frames.size(); i++) {
                if (frames[i] == frame) {
                    break;
                }
            }

            i = i + 2;

            int n = 0;
            for (; i < frames.size() && n < nFrames; i++, n++) {

                List<Camera> motionToTries = constantVelocityMotionModel(frames[i - 1]->getCamera(), frames[i]->getCamera());

                for (auto motionToTry : motionToTries) {

                    Camera camera = frames[i - 1]->getCamera();

                    for (int j = 0; j < n + 1; j++) {

                        camera = camera * motionToTry;

                    }

                    results.emplace_back(camera);

                }


            }


            return results;

        }

    private:
        void onFrameGroupChange(PFrame frame, int groupId, bool state) final;

        void onFrameDelete(PFrame frame) final;

        void onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) final;

        void onMapPointDestroyed(PPoint mapPoint) final;

    private:
        Mutex mMapPointsMutex;
        Set<PPoint> mMapPoints;

        Mutex mFramesMutex;
        OrderedSet<PFrame, Comparator> mFrames;

        GroupsManager mFrameGroupsManager, mMapPointsGroupManager;
        PrivateDataContext mFramePrivateDataContext, mMapPointsPrivateDataContext;

        std::array<OrderedSet<PFrame, Comparator>, MAXGROUPSIZE> mGroupsFrames;
        std::array<Set<PPoint>, MAXGROUPSIZE> mGroupsMapPoint;
        std::array<Mutex, MAXGROUPSIZE> mGroupsFrameMutexes;
        std::array<Mutex, MAXGROUPSIZE> mGroupsMapPointMutexes;


        Set<Observer*> mObservers;
        Mutex mObserversMutex;

        GarbageCollector mGarbageCollector;

        Mutex mErrorMutex;
        List<scalar_t> mATE;
        List<scalar_t> mRPE;
        List<Camera> mAlignedGronudtruth;

        Atomic<size_t> mFrameCounter, mGroupCounter[MAXGROUPSIZE];
        Atomic<size_t> mMapObjectCounter;

        Mutex mMapObjectAvailableIdsMutex;
        LinkedList<size_t> mMapObjectAvailableIds;

        Mutex mBuffersMutex;
        List<scalar_t*> mBufferCoordinates;
        List<scalar_t*> mBufferColors;
        List<scalar_t*> mBufferUncertainty;
        List<unsigned int*> mBufferGroups;

        List<scalar_t*> mBufferFrameCenter;
        List<int> mBufferFrameCenterSize;

    public:
        const int VALIDFRAME = createFrameGroup("Valid");
        const int KEYFRAME = createFrameGroup("Key");
        const int INITFRAME = createFrameGroup("Initial");
        // const int ACTIVEKEYFRAME = createFrameGroup("Active");
        const int HIGHLIGHTFRAME = createFrameGroup("Highlight");

        const int MAPPED = createMapPointGroup("Mapped");
        // const int ACTIVEMAPOBJECT = createMapPointGroup("Active");
        const int INDIRECTGROUP = createMapPointGroup("Indirect");
        const int DIRECTGROUP = createMapPointGroup("Direct");



    };

}


#endif //CML_MAP_H
