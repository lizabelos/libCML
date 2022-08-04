#ifndef CML_MAPOBJECT_H
#define CML_MAPOBJECT_H

#include <cml/config.h>
#include <cml/map/PrivateData.h>
#include <cml/map/Camera.h>
#include <cml/features/Features.h>
#include <cml/utils/GarbageCollector.h>
#include <cml/utils/PoolAllocator.h>

namespace CML {

#define CML_MAPPOINT_STOREPATCH 0
#define CML_MAPPOINT_STORECORNER 1
#define CML_MAPPOINT_SPARSITY 1
#define CML_MAPPOINT_STOREINDIRECTFRAME 1
#define CML_MAPPOINT_STOREDIRECTFRAME 1

    typedef struct {
        float data[7][7];
    } GrayPatch;

    typedef struct {
        Vector3f data[7][7];
    } GradientPatch;

    class MapPoint final : public Garbage {

        friend class Map;
        friend class Frame;

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
            virtual void onMapPointGroupChange(PPoint mapPoint, int groupId, bool state) {

            }

            virtual void onMapPointDestroyed(PPoint mapPoint) {

            }

            virtual void onAddIndirectApparition(PPoint mapPoint, PFrame frame) {

            }

            virtual void onRemoveIndirectApparition(PPoint mapPoint, PFrame frame) {

            }

            virtual void onAddDirectApparition(PPoint mapPoint, PFrame frame) {

            }

            virtual void onRemoveDirectApparition(PPoint mapPoint, PFrame frame) {

            }

        };

        MapPoint(size_t id, PFrame reference, FeatureIndex referenceIndex, MapPointType type, scalar_t *coordinate, scalar_t *color, scalar_t *uncertainty, unsigned int *groups);

        MapPoint(const MapPoint &) = delete;

        MapPoint &operator=(const MapPoint &) = delete;

        ~MapPoint() final;

        EIGEN_STRONG_INLINE std::size_t getId() const {
            return mId;
        }

        inline uint64_t getHash() const {
            return mHash;
        }

        /*EIGEN_STRONG_INLINE MapPointType getType() const {
            return (MapPointType)((mOptions >> 0) & (uint8_t)1);
        }*/

        EIGEN_STRONG_INLINE PFrame getReferenceFrame() const {
            return mReference;
        }

        Corner getReferenceCorner() const;

        scalar_t getReferenceInverseDepth() const;

        void setReferenceInverseDepth(scalar_t idepth);

        WorldPoint getWorldCoordinate() const;

        WorldPoint getWorldCoordinate(const Vector2 &shift) const;

        WorldPoint getWorldCoordinateIf(double idepth, const Vector2 &shift) const;

        void setWorldCoordinate(WorldPoint point);

#if CML_MAPPOINT_STOREINDIRECTFRAME
        EIGEN_STRONG_INLINE List<PFrame> getIndirectApparitions() const {
            LockGuard lg(mApparitionsMutex);
            List<PFrame> result;
            result.reserve(mApparitions.size());
            for (auto [frame, mode] : mApparitions) {
                bool isIndirect = (mode >> 1) & 1U;
                if (isIndirect) {
                    result.emplace_back(frame);
                }
            }
            return result;
        }

        EIGEN_STRONG_INLINE void getIndirectApparitions(List<PFrame> &result) const {
            LockGuard lg(mApparitionsMutex);
            result.clear();
            result.reserve(mApparitions.size());
            for (auto [frame, mode] : mApparitions) {
                bool isIndirect = (mode >> 1) & 1U;
                if (isIndirect) {
                    result.emplace_back(frame);
                }
            }
        }

        EIGEN_STRONG_INLINE bool haveIndirectApparition(PFrame frame) {
            LockGuard lg(mApparitionsMutex);
            if (mApparitions.contains(frame)) {
                bool isIndirect = (mApparitions[frame] >> 1) & 1U;
                return isIndirect;
            }
            return false;
        }
#endif

        EIGEN_STRONG_INLINE unsigned short getIndirectApparitionNumber() const {
            return mIndirectApparitionsNumber;
        }

#if CML_MAPPOINT_STOREDIRECTFRAME
        EIGEN_STRONG_INLINE List<PFrame> getDirectApparitions() const {
            LockGuard lg(mApparitionsMutex);
            List<PFrame> result;
            result.reserve(mApparitions.size());
            for (auto [frame, mode] : mApparitions) {
                bool isDirect = (mode >> 2) & 1U;
                if (isDirect) {
                    result.emplace_back(frame);
                }
            }
            return result;
        }

        EIGEN_STRONG_INLINE void getDirectApparitions(List<PFrame> &result) const {
            LockGuard lg(mApparitionsMutex);
            result.reserve(mApparitions.size());
            for (auto [frame, mode] : mApparitions) {
                bool isDirect = (mode >> 2) & 1U;
                if (isDirect) {
                    result.emplace_back(frame);
                }
            }
        }
#endif

        EIGEN_STRONG_INLINE unsigned short getDirectApparitionNumber() const {
            return mDirectApparitionsNumber;
        }

        EIGEN_STRONG_INLINE bool operator==(const MapPoint &other) const {
            return other.mId == mId;
        }

        EIGEN_STRONG_INLINE PrivateData &getPrivate() {
            return mPrivate;
        }

        scalar_t getGrayPatch(int x, int y, int level) const;

        Vector3f getDerivativePatch(int x, int y, int level) const;

        EIGEN_STRONG_INLINE Vector3 getColor() const {
            return Vector3(mColor[0], mColor[1], mColor[2]);
        }

        void setMarginalized(bool value) {
            mOptions ^= (-value ^ mOptions) & ((uint8_t)1 << 2);
        }

        bool isMarginalized() const {
            return (mOptions >> 2) & (uint8_t)1;
        }

        void setGroup(int groupId, bool state);

        [[nodiscard]] inline bool isGroup(int groupId) const {
            if (groupId == -1) {
                return true;
            }
            return ((*mGroups) >> groupId) & 1U;
        }

        inline int getGroups() const {
            return *mGroups;
        }

        inline void subscribeObserver(Observer *observer) {
            assertThrow(observer != nullptr, "null observer passed to subscribeObserver");
            mObservers.emplace_back(observer);
            for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
                if (isGroup(i)) {
                    observer->onMapPointGroupChange(this, i, true);
                }
            }
        }

        inline void unsafe_slow_removeObserver(Observer *observer) {
            auto i = mObservers.begin();
            while (i != mObservers.end()) {
                auto currentItem = *i;
                if (currentItem == observer) {
                    mObservers.erase(i);
                    return;
                }
                i++;
            }
        }

        inline PFrame getLastSeen() const {
            if (mLastSeen.isNull()) return mReference;
            return mLastSeen;
        }

        inline void setUncertainty(scalar_t v) {
            for (int i = 0; i < CML_MAPPOINT_SPARSITY; i++) {
                mUncertainty[i] = v;
            }
        }

        inline scalar_t getUncertainty() const {
            return *mUncertainty;
        }

        bool isReferenceDepth() const {
            return (mOptions >> 1) & (uint8_t)1;
        }

        void freeDescriptor() {
            if (mDescriptor != nullptr) {
                delete mDescriptor.p();
                mDescriptor = nullptr;
            }
        }

        template <typename D> void setDescriptor(const D &descriptor) {
            freeDescriptor();
            mDescriptor = new D(descriptor);
        }

        template <typename D> const D &getDescriptor() {
            assertThrow(mDescriptor != nullptr, "The descriptor of this point has not been set");
            return *((D*)mDescriptor.p());
        }

    protected:
        void onCameraChange(const Camera &camera) {
            // update reference depth
            if (isReferenceDepth()) {
                setReferenceInverseDepth(getReferenceInverseDepth());
            }
        }

        void clearApparitions() {
#if CML_MAPPOINT_STOREINDIRECTFRAME || CML_MAPPOINT_STOREDIRECTFRAME
            LockGuard lg(mApparitionsMutex);
            mApparitions.clear();
#endif
            mIndirectApparitionsNumber = 0;
            mDirectApparitionsNumber = 0;
        }

    protected:
        void addIndirectApparition(PFrame frame, const Corner &where);

        void removeIndirectApparition(PFrame frame);

        void addDirectApparition(PFrame frame);

        void removeDirectApparition(PFrame frame);

    private:
        PrivateData mPrivate;

        mutable Mutex mApparitionsMutex;
#if CML_MAPPOINT_STOREINDIRECTFRAME || CML_MAPPOINT_STOREDIRECTFRAME
        HashMap<OptPFrame, uint8_t> mApparitions;
#endif
        LinkedList<Ptr<Observer, NonNullable>> mObservers;
#if CML_MAPPOINT_STOREPATCH
        List<GrayPatch> mGrayPatchs;
        List<GradientPatch> mGradientPatchs;
#endif
#if CML_MAPPOINT_STORECORNER
        Corner mReferenceCorner;
#endif

        scalar_t mReferenceInverseDepth = 0;

        size_t mId;
        uint64_t mHash;

        PFrame mReference;
        OptPFrame mLastSeen;
        Ptr<Descriptor, Nullable> mDescriptor;
        unsigned int *mGroups;
        scalar_t *mWorldCoordinate;
        scalar_t *mUncertainty;
        scalar_t *mColor;

        FeatureIndex mReferenceFeatureIndex;

        unsigned short mDirectApparitionsNumber = 0, mIndirectApparitionsNumber = 0;

        // Bit 0 : Map Point Type
        // Bit 1 : Is Reference Depth
        // Bit 2 : Is Marginalized
        uint8_t mOptions; // 1 bytes

    };

    inline size_t Hasher::operator()(PPoint pPoint) const {
#if CML_FULLY_REPRODUCIBLE
        return pPoint->getId();
#else
        return reinterpret_cast<size_t>(pPoint.p());
#endif
    }

    inline bool CML::Comparator::operator() (PPoint pPointA, PPoint pPointB) const {
        return pPointA->getId() > pPointB->getId();
    }


}

#include <cml/map/Frame.h>

namespace CML {

    EIGEN_STRONG_INLINE void MapPoint::setGroup(int groupId, bool state) {
        assertThrow(groupId >= 0 && groupId < MAPOBJECT_GROUP_MAXSIZE, "Invalid group id");
        if (groupId != 0 && !isGroup(0) && state == true) {
            return;
        }
        if (isGroup(groupId) != state) {
            (*mGroups) ^= (-(unsigned int) state ^ (*mGroups)) & (1U << groupId);
            for (auto observer : mObservers) {
                observer->onMapPointGroupChange(this, groupId, state);
            }
            for (auto frame : getIndirectApparitions()) {
                frame->onMapPointGroupChange(this, groupId, state);
            }
            for (auto frame : getDirectApparitions()) {
                frame->onMapPointGroupChange(this, groupId, state);
            }
            for (int i = 1; i < CML_MAPPOINT_SPARSITY; i++) {
                mGroups[i] = mGroups[0];
            }
        }
    }

    EIGEN_STRONG_INLINE Corner MapPoint::getReferenceCorner() const {
#if CML_MAPPOINT_STORECORNER
        return mReferenceCorner;
#else
        return getReferenceFrame()->getFeaturePoint(mReferenceFeatureIndex);
#endif
    }

    EIGEN_STRONG_INLINE scalar_t MapPoint::getGrayPatch(int x, int y, int level) const {
#if CML_MAPPOINT_STOREPATCH
        assertThrow(std::abs(x) <= 3 && std::abs(y) <= 3, "Out of range patch");
        assertThrow(level < mGrayPatchs.size(), "Invalid level : " + std::to_string(level) + ". Have" + std::to_string(mGradientPatchs.size()));
        return mGrayPatchs[level].data[3 + x][3 + y];
#else
        auto p = (Eigen::Vector2i) (getReferenceCorner().point(level).cast<int>() + Eigen::Vector2i(x, y));
        return mReference->getCaptureFrame().getGrayImage(level).get(p.x(), p.y());
#endif
    }

    EIGEN_STRONG_INLINE Vector3f MapPoint::getDerivativePatch(int x, int y, int level) const {
#if CML_MAPPOINT_STOREPATCH
        assertThrow(std::abs(x) <= 3 && std::abs(y) <= 3, "Out of range patch");
        assertThrow(level < mGradientPatchs.size(), "Invalid level : " + std::to_string(level) + ". Have" + std::to_string(mGradientPatchs.size()));
        return mGradientPatchs[level].data[3 + x][3 + y];
#else
        auto p = (Eigen::Vector2i) (getReferenceCorner().point(level).cast<int>() + Eigen::Vector2i(x, y));
        return mReference->getCaptureFrame().getDerivativeImage(level).get(p.x(), p.y());
#endif
    }

}

#endif //CML_MAPOBJECT_H
