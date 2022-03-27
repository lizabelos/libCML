#ifndef CML_FRAME_H
#define CML_FRAME_H

#include <cml/config.h>

#include <memory>

#include <cml/capture/CaptureImage.h>
#include <cml/features/Features.h>
#include <cml/features/bow/Bow.h>
#include <cml/map/Camera.h>
#include <cml/map/InternalCalibration.h>
#include <cml/map/PrivateData.h>
#include <cml/map/Exposure.h>
#include <cml/utils/GarbageCollector.h>


namespace CML {

    class Frame : public EvaluationFrame, public Garbage {

        friend class Map;
        friend class MapPoint;

    public:
        static PoolAllocator allocator;

        static void *operator new(size_t size) {
            return allocator.allocate(size);
        }

        static void operator delete(void *ptr, size_t size) {
            return allocator.deallocate(ptr, size);
        }

        class Observer : public DeterministicallyHashable {

        public:
            virtual void onFrameGroupChange(PFrame frame, int groupId, bool state) {

            }

            virtual void onFrameDelete(PFrame frame) {

            }

            virtual void onCameraChanged(PFrame frame, const Camera &camera) {

            }

            virtual void onExposureChanged(PFrame frame, const Exposure &exposure) {

            }

        };

        struct Deform {

            Deform(PFrame _frame0, PFrame _frame1) : frame0(_frame0), frame1(_frame1) {

            }

            PFrame frame0;
            PFrame frame1;
            Camera camera;

        };

        Frame(size_t id, Ptr<CaptureImage, NonNullable> captureFrame, scalar_t *cameraCenter) : mId(id), mCaptureFrame(captureFrame), mCalibration(captureFrame->getInternalCalibration()), mExposure(captureFrame->getExposureTime()), mCameraCenter(cameraCenter) {

#if CML_USE_GOOGLE_HASH
            mIndirectCovisibility.set_empty_key(OptPFrame((Frame*)0));
            mIndirectCovisibility.set_deleted_key(OptPFrame((Frame*)1));

            mDirectCovisibility.set_empty_key(OptPFrame((Frame*)0));
            mDirectCovisibility.set_deleted_key(OptPFrame((Frame*)1));
#endif

            for (int i = 0; i < FRAME_GROUP_MAXSIZE; i++) {
                mGroups[i] = false;
            }
            mCameraCenter[0] = 0;
            mCameraCenter[1] = 0;
            mCameraCenter[2] = 0;
        }

        virtual ~Frame() {
            LockGuard lg(mObserversMutex);
            for (Observer *observer : mObservers) {
                observer->onFrameDelete(this);
            }
        }

        Frame( const Frame& ) = delete;
        // Frame& operator=( const Frame& ) = delete;

        inline size_t getId() const {
            return mId;
        }

        inline void setGroupId(int group, size_t id) {
            mGroupId[group] = id;
        }

        inline size_t getGroupId(int group) const {
            return mGroupId[group];
        }

        inline void subscribeObserver(Observer *observer) {
            assertThrow(observer != nullptr, "null observer passed to subscribeObserver");
            LockGuard lg(mObserversMutex);
            mObservers.insert(observer);
            for (int i = 0; i < FRAME_GROUP_MAXSIZE; i++) {
                if (mGroups[i]) {
                    observer->onFrameGroupChange(this, i, true);
                }
            }
        }

        inline void removeObserver(Observer *observer) {
            LockGuard lg(mObserversMutex);
            mObservers.erase(observer);
        }

        [[nodiscard]] inline unsigned int getWidth(int level) const {
            return mCaptureFrame->getWidth(level);
        }

        [[nodiscard]] inline unsigned int getHeight(int level) const {
            return mCaptureFrame->getHeight(level);
        }

        [[nodiscard]] inline bool isInside(Vector2 point, int level, scalar_t padding) const {
            return point.x() >= padding && point.y() >= padding && point.x() < getWidth(level) - padding && point.y() < getHeight(level) - padding;
        }

        inline CaptureImage& getCaptureFrame() {
            return *mCaptureFrame.p();
        }

        inline void setGroup(int groupId, bool state) {
            assertThrow(groupId >= 0 && groupId < FRAME_GROUP_MAXSIZE, "Invalid group id");
            if (mGroups[groupId] != state) {

                /*if (groupId == KEYFRAME) {
                    mCaptureFrame->makeKey();
                } */ // Move move this into Map.h

                mGroups[groupId] = state;
                for (Observer *observer : mObservers) {
                    observer->onFrameGroupChange(this, groupId, state);
                }
            }
        }

        [[nodiscard]] inline bool isGroup(int groupId) const {
            if (groupId == -1) {
                return true;
            }
            return mGroups[groupId];
        }

        [[nodiscard]] inline const CML::InternalCalibration& getCalibration() const {
            return mCalibration;
        }

        inline const Camera& getCamera() const {
            return mCamera;
        }

        void setCamera(const Camera &camera, bool updateDeforms = false);

        static void setCameraAndDeform(const HashMap<PFrame, Camera> &frames);

        static void setCameraAndDeform(const HashMap<PFrame, Camera> &frames, Set<PFrame> &skip);

        void setCameraWithoutObserver(const Camera &camera);

        [[nodiscard]] inline List<Pair<FeatureIndex, PPoint>> getMapPoints() const {
            LockGuard lg(mMapPointsMutex);
            List<Pair<FeatureIndex, PPoint>> l;
            for (size_t groupId = 0; groupId < mMapPoints.size(); groupId++) {
                for (size_t index = 0; index < mMapPoints[groupId].size(); index++) {
                    if (mMapPoints[groupId][index].isNotNull()) {
                        l.emplace_back(FeatureIndex(groupId, index), mMapPoints[groupId][index]);
                    }
                }
            }
            return l;
        }

        [[nodiscard]] inline List<PPoint> getMapPointsWithoutIndex() const {
            LockGuard lg(mMapPointsMutex);
            List<PPoint> l;
            for (size_t groupId = 0; groupId < mMapPoints.size(); groupId++) {
                for (size_t index = 0; index < mMapPoints[groupId].size(); index++) {
                    if (mMapPoints[groupId][index].isNotNull()) {
                        l.emplace_back(mMapPoints[groupId][index]);
                    }
                }
            }
            return l;
        }

        inline OrderedSet<PPoint, Comparator> getOrderedMapPoints() const {
            LockGuard lg(mMapPointsMutex);
            return mOrderedMapPoints;
        }

        [[nodiscard]] inline List<Pair<FeatureIndex, PPoint>> getMapPoints(int groupId) const {
            LockGuard lg(mMapPointsMutex);
            List<Pair<FeatureIndex, PPoint>> l;
            for (size_t index = 0; index < mMapPoints[groupId].size(); index++) {
                if (mMapPoints[groupId][index].isNotNull()) {
                    l.emplace_back(FeatureIndex(groupId, index), mMapPoints[groupId][index]);
                }
            }
            return l;
        }

        [[nodiscard]] inline List<Corner> getFeaturePoints(int group) const {
            // assertThrow(group >= 0 && group < mFeaturePoints.size(), "No a valid group : " + std::to_string(group));
            if (group < 0 || group > mFeaturePoints.size()) {
                return {};
            }
            LockGuard lg(mFeatureMutex);
            return mFeaturePoints[group];
        }

        [[nodiscard]] inline Corner getFeaturePoint(FeatureIndex index) const {
            LockGuard lg(mFeatureMutex);
            assertThrow(index.hasValidValue(), "Index don't have a valid value !");
            assertThrow(index.group >= 0 && index.group < (int)mFeaturePoints.size(), "Invalid group : " + std::to_string(index.group) + ". Max group : " + std::to_string(mFeaturePoints.size()));
            assertThrow(index.index >= 0 && index.index < (int)mFeaturePoints[index.group].size(), "Invalid index : " + std::to_string(index.index) + ". Max index : " + std::to_string(mFeaturePoints[index.group].size()));
            return mFeaturePoints[index.group][index.index];
        }

        [[nodiscard]] inline Optional<Corner> getFeaturePoint(PPoint mapPoint) const {
            FeatureIndex i = getIndex(mapPoint);
            if (!i.hasValidValue()) {
                return {};
            } else {
                LockGuard lg(mFeatureMutex);
                return {mFeaturePoints[i.group][i.index]};
            }
        }

        List<NearestNeighbor> processNearestNeighbors(int group, Vector2 position, int num) const;

        List<NearestNeighbor> processNearestNeighborsInRadius(int group, Vector2 position, float distance) const;



        [[nodiscard]] inline OptPPoint getMapPoint(FeatureIndex i) const {
            assertThrow(i.hasValidValue() && i.group < (int)mMapPoints.size() && i.index < (int)mMapPoints[i.group].size(), "Invalid feature index : " + std::to_string(i.group)  + "; " + std::to_string(i.index));
            //LockGuard lg(mMapPointsMutex);
            return mMapPoints[i.group][i.index];
        }

        [[nodiscard]] inline FeatureIndex getIndex(PPoint mapPoint) const {
            LockGuard lg(mMapPointsMutex);
            auto search = mMapPointsIndex.find(mapPoint);
            if (search == mMapPointsIndex.end()) {
                return FeatureIndex();
            }
            else {
                return search->second;
            }
        }

        inline List<FeatureIndex> getIndexes(const List<PPoint> &mapPoints) const {
            List<FeatureIndex> indexes;
            indexes.resize(mapPoints.size());
            LockGuard lg(mMapPointsMutex);
            for (auto [point, index] : mMapPointsIndex) {
                for (int i = 0; i < mapPoints.size(); i++) {
                    if (mapPoints[i] == point) {
                        indexes[i] = index;
                    }
                }
            }
            /*for (int i = 0; i < mapPoints.size(); i++) {
                if (mMapPointsIndex.count(mapPoints[i]) > 0) {
                    indexes[i] = mMapPointsIndex.at(mapPoints[i]);
                }
            }*/
            return indexes;
        }

        EIGEN_STRONG_INLINE DistortedVector2d distort(const UndistortedVector2d &input, int level) const {
            return mCalibration.distort(input, level);
        }

        EIGEN_STRONG_INLINE UndistortedVector2d undistort(const DistortedVector2d &input, int level) const {
            return mCalibration.undistort(input, level);
        }

        Matrix33 getK(int level) {
            return mCalibration.getK(level);
        }

        int addFeaturePoints(const List<Corner> &features, Ptr<Features::BoW, Nullable> bow = nullptr);

        void setBow(int group, Ptr<Features::BoW, Nullable> bow) {
            mBoW[group] = bow;
        }

        bool setMapPoint(FeatureIndex i, OptPPoint mapPoint);

        void removeMapPoint(PPoint mapPoint);

        void addDirectApparitions(OptPPoint mapPoint);

        Set<PPoint> getMapPointsApparitions() {
            LockGuard lg(mMapPointsApparitionsMutex);
            return mMapPointsApparitions;
        }

        inline size_t getId() {
            return mId;
        }

        inline bool operator<(const Frame &other) const {
            return other.mId < mId;
        }

        inline bool operator==(const Frame &other) const {
            return other.mId == mId;
        }

        inline void setReference(PFrame frame) {
            mReference = frame;
        }

        PFrame getReference() {
            return mReference;
        }

        inline scalar_t getEvaluationTimestamp() final {
            return mCaptureFrame->getEvaluationTimestamp();
        }

        Camera getEvaluationPosition() final;

        FloatImage *& getDepthMap() {
            return mDepthMap;
        }

        PrivateData &getPrivateData() {
            return mPrivataData;
        }

        void setJacobianParameterId(int id) {
            mJacobianParameterId = id;
        }

        int getJacobianParameterId() const {
            return mJacobianParameterId;
        }

        inline const Exposure& getExposure() const {
            return mExposure;
        }

        inline void setExposureParameters(const Exposure &exposure) {
            // assertThrow(exposure.getExposureFromCamera() == mExposure.getExposureFromCamera(), "Different exposure from camera");
            mExposure.setParameters(exposure);
            for (Observer *observer : mObservers) {
                observer->onExposureChanged(this, mExposure);
            }
        }

        int numFeaturesGroup() {
            LockGuard lg(mFeatureMutex);
            return mFeaturePoints.size();
        }

        Ptr<Features::BoW, Nullable> getBoW(int group) {
            LockGuard lg(mFeatureMutex);
            assertThrow(group < (int)mBoW.size(), "Invalid BoW group");
            return mBoW[group];
        }

        bool haveBoW(int group) {
            LockGuard lg(mFeatureMutex);
            return mBoW[group].isNotNull();
        }

        Set<PPoint> getGroupMapPoints(int groupId) {
            assertThrow(groupId >= 0, "Invalid group id");
            LockGuard lg(mGroupsMapPointMutexes[groupId]);
            return mGroupsMapPoint[groupId];
        }

        void getGroupMapPointsNoClean(int groupId, Set<PPoint> &output) {
            assertThrow(groupId >= 0, "Invalid group id");
            LockGuard lg(mGroupsMapPointMutexes[groupId]);
            for (auto point : mGroupsMapPoint[groupId]) {
                output.insert(point);
            }
        }

        Set<PPoint> getReferenceGroupMapPoints(int groupId) {
            LockGuard lg(mGroupsMapPointReferenceMutexes[groupId]);
            return mGroupsMapPointReference[groupId];
        }

        void addDeform(const Deform &deform) {
            {
                LockGuard lg(mDeformsMutex);
                mDeforms.emplace_back(deform);
            }

            PFrame thisPtr(this);

            deform.frame0->mToDeformMuex.lock();
            deform.frame0->mToDeform.insert(thisPtr);
            deform.frame0->mToDeformMuex.unlock();

            deform.frame1->mToDeformMuex.lock();
            deform.frame1->mToDeform.insert(thisPtr);
            deform.frame1->mToDeformMuex.unlock();
        }

        void resetDeforms() {
            LockGuard lg(mDeformsMutex);
            PFrame thisPtr(this);
            for (auto deform : mDeforms) {
                deform.frame0->mToDeformMuex.lock();
                deform.frame0->mToDeform.erase(thisPtr);
                deform.frame0->mToDeformMuex.unlock();

                deform.frame1->mToDeformMuex.lock();
                deform.frame1->mToDeform.erase(thisPtr);
                deform.frame1->mToDeformMuex.unlock();
            }
            mDeforms.clear();
        }

        void addDeform(PFrame frame0, PFrame frame1) {
            addDeformSingleDirection(frame0, frame1);
            addDeformSingleDirection(frame1, frame0);
        }

        int shared(int groupId, PFrame other);

        void setParent(PFrame parent) {
            if (mParent.isNotNull()) {
                mParent->mChilds.erase(PFrame(this));
            }
            parent->mChilds.insert(PFrame(this));
            mParent = parent;
        }

        OptPFrame getParent() {
            return mParent;
        }

        Set<PFrame> getChilds() {
            return mChilds;
        }

        scalar_t computeMedianDepth(bool useDirect = false, bool useIndirect = true);

    protected:
        void onMapPointErased(PPoint mapPoint);

        Camera computeNewCameraFromDeforms();

        void addDeformSingleDirection(PFrame frame0, PFrame frame1);

        EIGEN_STRONG_INLINE void onAddIndirectApparition(PPoint mapPoint, PFrame frame) {
            LockGuard lg(mIndirectCovisibilityMutex);
            if (mIndirectCovisibility.count(frame) == 0) {
                mIndirectCovisibility[frame] = 1;
            } else {
                mIndirectCovisibility[frame]++;
            }
        }

        EIGEN_STRONG_INLINE void onRemoveIndirectApparition(PPoint mapPoint, PFrame frame) {
            LockGuard lg(mIndirectCovisibilityMutex);
            if (mIndirectCovisibility.count(frame) == 0) {
                // todo : strange
            } else {
                mIndirectCovisibility[frame]--;
                if (mIndirectCovisibility[frame] == 0) {
                    mIndirectCovisibility.erase(frame);
                }
            }
        }

        EIGEN_STRONG_INLINE void onAddDirectApparition(PPoint mapPoint, PFrame frame) {
            LockGuard lg(mDirectCovisibilityMutex);
            if (mDirectCovisibility.count(frame) == 0) {
                mDirectCovisibility[frame] = 1;
            } else {
                mDirectCovisibility[frame]++;
            }
        }

        EIGEN_STRONG_INLINE void onRemoveDirectApparition(PPoint mapPoint, PFrame frame) {
            LockGuard lg(mDirectCovisibilityMutex);
            if (mDirectCovisibility.count(frame) == 0) {
                // todo : strange
            } else {
                mDirectCovisibility[frame]--;
                if (mDirectCovisibility[frame] == 0) {
                    mDirectCovisibility.erase(frame);
                }
            }
        }

        EIGEN_STRONG_INLINE DenseHashMap<OptPFrame, int> getIndirectCovisibilities() {
            LockGuard lg(mIndirectCovisibilityMutex);
            return mIndirectCovisibility;
        }

        EIGEN_STRONG_INLINE DenseHashMap<OptPFrame, int> getDirectCovisibilities() {
            LockGuard lg(mDirectCovisibilityMutex);
            return mDirectCovisibility;
        }


    private:
        void onMapPointGroupChange(PPoint mapPoint, int groupId, bool state);

    private:
        size_t mId;
        size_t mGroupId[FRAME_GROUP_MAXSIZE];

        int mJacobianParameterId = 0;

        bool mGroups[FRAME_GROUP_MAXSIZE];

        Ptr<CaptureImage, NonNullable> mCaptureFrame;
        InternalCalibration &mCalibration;

        Camera mCamera;
        Exposure mExposure;
        scalar_t *mCameraCenter;
        List<Ptr<Features::BoW, Nullable>> mBoW;
        Vector3 mGroundtruth;

        OptPFrame mReference = OptPFrame();

        Set<Observer*> mObservers;
        Mutex mObserversMutex;

        List<List<Corner>> mFeaturePoints;
        mutable Mutex mFeatureMutex;

        List<std::shared_ptr<PointGrid<Corner>>> mFeaturePointTree;

        //HashMap<FeatureIndex, PPoint, FeatureIndex> mMapPoints;
        List<List<OptPPoint>> mMapPoints;
        OrderedSet<PPoint, Comparator> mOrderedMapPoints;
        HashMap<PPoint, FeatureIndex> mMapPointsIndex;
        mutable Mutex mMapPointsMutex;

        Set<PPoint> mMapPointsApparitions;
        mutable Mutex mMapPointsApparitionsMutex;

        FloatImage *mDepthMap = nullptr;

        PrivateData mPrivataData;

        std::array<Set<PPoint>, MAPOBJECT_GROUP_MAXSIZE> mGroupsMapPoint;
        std::array<Mutex, MAPOBJECT_GROUP_MAXSIZE> mGroupsMapPointMutexes;

        std::array<Set<PPoint>, MAPOBJECT_GROUP_MAXSIZE> mGroupsMapPointReference;
        std::array<Mutex, MAPOBJECT_GROUP_MAXSIZE> mGroupsMapPointReferenceMutexes;

        Mutex mDeformsMutex;
        List<Deform> mDeforms;

        Mutex mToDeformMuex;
        Set<PFrame> mToDeform;

        OptPFrame mParent;
        Set<PFrame> mChilds;

        Mutex mIndirectCovisibilityMutex, mDirectCovisibilityMutex;
        DenseHashMap<OptPFrame, int> mIndirectCovisibility, mDirectCovisibility;

    };


}



#endif //CML_FRAME_H
