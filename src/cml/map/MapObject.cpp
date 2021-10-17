//
// Created by tbelos on 19/04/19.
//

#include "cml/map/MapObject.h"
#include "cml/utils/Watchpoint.h"

CML::PoolAllocator CML::MapPoint::allocator{16384};

CML::MapPoint::MapPoint(size_t id, PFrame reference, FeatureIndex referenceIndex, MapPointType type, scalar_t *coordinate, scalar_t *color, scalar_t *uncertainty, unsigned int *groups)
: mId(id), mReference(reference), mReferenceFeatureIndex(referenceIndex), mWorldCoordinate(coordinate), mUncertainty(uncertainty), mColor(color), mGroups(groups)
{
    assertThrow(reference != nullptr, "with a null reference pointer");

#if CML_USE_GOOGLE_HASH
    mApparitions.set_empty_key(OptPFrame((Frame*)0));
    mApparitions.set_deleted_key(OptPFrame((Frame*)1));
#endif


//    assertThrow(reference->isInside(corner.point(0), 0, 0), "corner is not inside the reference");

    mOptions = 0;
    mOptions ^= (-((int)type) ^ mOptions) & ((uint8_t)1 << 0);

#if CML_MAPPOINT_STORECORNER
    mReferenceCorner = reference->getFeaturePoint(mReferenceFeatureIndex);
#endif


    // reference->getCamera().subscribeObserver(this);

    switch (type) {
        case DIRECT:
            break;
        case INDIRECT:
            break;
        default:
            assertThrow(false, "invalid type");
    }

    for (int i = 0; i < CML_MAPPOINT_SPARSITY; i++) {

        if (mReference->getCaptureFrame().haveColorImage()) {
            auto p = (Eigen::Vector2i) (getReferenceCorner().point(0) + PredefinedPattern::star9(i)).cast<int>();
            Vector3 colorRGB = mReference->getCaptureFrame().getColorImage(0).get(p.x(), p.y()).eigen().cast<scalar_t>();
            //  float colorGray = mReference->getCaptureFrame().getGrayImage(0).get(p.x(), p.y());
            // colorRGB = colorRGB / colorRGB.norm() * colorGray;
            mColor[i * 3 + 0] = colorRGB[0] / 255.0f;
            mColor[i * 3 + 1] = colorRGB[1] / 255.0f;
            mColor[i * 3 + 2] = colorRGB[2] / 255.0f;

        } else {
            auto p = (Eigen::Vector2i) (getReferenceCorner().point(0) + PredefinedPattern::star9(i)).cast<int>();
            float colorGray = mReference->getCaptureFrame().getGrayImage(0).get(p.x(), p.y());
            mColor[i * 3 + 0] = colorGray / 255.0f;
            mColor[i * 3 + 1] = colorGray / 255.0f;
            mColor[i * 3 + 2] = colorGray / 255.0f;
        }

        mGroups[i] = 0;
        mUncertainty[i] = 1000000;

    }


#if CML_MAPPOINT_STOREPATCH
    mGrayPatchs.resize(reference->getCaptureFrame().getPyramidLevels());
    mGradientPatchs.resize(reference->getCaptureFrame().getPyramidLevels());

    for (int level = 0; level < reference->getCaptureFrame().getPyramidLevels(); level++) {

        for (int x = -3; x <= 3; x++) {

            for (int y = -3; y <= 3; y++) {

                if (!reference->isInside(DistortedVector2d(getReferenceCorner().point(level) + Vector2(x, y)), level, 0)) {
                    mGrayPatchs[level].data[3 + x][3 + y] = 0;
                    mGradientPatchs[level].data[3 + x][3 + y] = Vector3f(0, 0, 0);
                    continue;
                }

                auto p = (Eigen::Vector2i) (getReferenceCorner().point(level).cast<int>() + Eigen::Vector2i(x, y));
                mGrayPatchs[level].data[3 + x][3 + y] = reference->getCaptureFrame().getGrayImage(level).get(p.x(), p.y());
                mGradientPatchs[level].data[3 + x][3 + y] = reference->getCaptureFrame().getDerivativeImage(level).get(p.x(), p.y());

            }

        }

    }
#endif

}

CML::MapPoint::~MapPoint() {

    for (auto [frame, mode] : mApparitions) {
        bool isIndirect = (mode >> 1) & 1U;
        bool isDirect = (mode >> 2) & 1U;
        if (isIndirect) {
            for (auto observer: mObservers) {
                observer->onRemoveIndirectApparition(this, frame);
            }
            for (auto [observer, mode] : mApparitions) {
                observer->onRemoveIndirectApparition(this, frame);
            }
        }
        if (isDirect) {
            for (auto observer: mObservers) {
                observer->onRemoveDirectApparition(this, frame);
            }
            for (auto [observer, mode] : mApparitions) {
                observer->onRemoveDirectApparition(this, frame);
            }
        }
    }
    for (auto observer : mObservers) {
        observer->onMapPointDestroyed(this);
    }
    freeDescriptor();
}

CML::scalar_t CML::MapPoint::getReferenceInverseDepth() const {
    if (isReferenceDepth()) {
        return mReferenceInverseDepth;
    } else {
        return 1.0 / getWorldCoordinate().relative(mReference->getCamera()).z();
    }
}

void CML::MapPoint::setReferenceInverseDepth(scalar_t idepth) {
    assertThrow(std::isfinite(idepth), "Non finite depth");
    assertThrow(idepth > 0, "Negative depth");
    mReferenceInverseDepth = idepth;
    for (int i = 0; i < CML_MAPPOINT_SPARSITY; i++) {
        Vector3 worldCoordinate = WorldPoint::fromInverseDepth(mReferenceInverseDepth, mReference->undistort(DistortedVector2d(getReferenceCorner().point0() +  PredefinedPattern::star9(i)), 0), mReference->getCamera()).absolute();
        mWorldCoordinate[3 * i + 0] = worldCoordinate[0];
        mWorldCoordinate[3 * i + 1] = worldCoordinate[1];
        mWorldCoordinate[3 * i + 2] = worldCoordinate[2];
    }
    mOptions ^= (-(unsigned int) true ^ mOptions) & (1U << 1);
    assertThrow(isReferenceDepth(), "Coding error in MapObject");
}

CML::WorldPoint CML::MapPoint::getWorldCoordinate() const {
    return WorldPoint::fromAbsolute(Vector3(mWorldCoordinate[0], mWorldCoordinate[1], mWorldCoordinate[2]));
}

CML::WorldPoint CML::MapPoint::getWorldCoordinate(const Vector2 &shift) const {
    return WorldPoint::fromInverseDepth(mReferenceInverseDepth, mReference->undistort(DistortedVector2d(getReferenceCorner().point0() + shift), 0), mReference->getCamera());
}

CML::WorldPoint CML::MapPoint::getWorldCoordinateIf(double idepth, const Vector2 &shift) const {
    return WorldPoint::fromInverseDepth(idepth, mReference->undistort(DistortedVector2d(getReferenceCorner().point0() + shift), 0), mReference->getCamera());
}

void CML::MapPoint::setWorldCoordinate(WorldPoint worldCoordinate) {
    // assertThrow(mReference->getCamera().inFront(point.absolute()), "The point must be in front of the reference camera");
    for (int i = 0; i < CML_MAPPOINT_SPARSITY; i++) {
        mWorldCoordinate[3 * i + 0] = worldCoordinate.absolute()[0];
        mWorldCoordinate[3 * i + 1] = worldCoordinate.absolute()[1];
        mWorldCoordinate[3 * i + 2] = worldCoordinate.absolute()[2];
    }
    mOptions ^= (-(unsigned int) false ^ mOptions) & (1U << 1);
    assertThrow(!isReferenceDepth(), "Coding error in MapObject");
}

void CML::MapPoint::addIndirectApparition(PFrame frame, const Corner &where) {
//    assertThrow(mIndirectApparitions.count(frame) == 0, "Already added the frame");
#if CML_MAPPOINT_STOREINDIRECTFRAME
    {
        LockGuard lg(mApparitionsMutex);
        if (mApparitions.count(frame) == 0) {
            mApparitions[frame] = 0;
        }
        uint8_t &mode = mApparitions[frame];
        mode ^= (-(unsigned int) true ^ mode) & (1U << 1);
    }
#endif
    if (mLastSeen.isNull()) mLastSeen = frame;
    else if (mLastSeen->getId() < frame->getId()) mLastSeen = frame;
    // mIndirectApparitions[frame] = where;

    mIndirectApparitionsNumber++;

    for (auto observer: mObservers) {
        observer->onAddIndirectApparition(this, frame);
    }
    for (auto [observer, mode] : mApparitions) {
        observer->onAddIndirectApparition(this, frame);
    }

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        if (isGroup(i)) {
            frame->onMapPointGroupChange(this, i, true);
        }
    }


}

void CML::MapPoint::addDirectApparition(PFrame frame) {
#if CML_MAPPOINT_STOREDIRECTFRAME
    {
        LockGuard lg(mApparitionsMutex);
        if (mApparitions.count(frame) == 0) {
            mApparitions[frame] = 0;
        }
        uint8_t &mode = mApparitions[frame];
        mode ^= (-(unsigned int) true ^ mode) & (1U << 2);
    }
#endif

    mDirectApparitionsNumber++;

    for (auto observer: mObservers) {
        observer->onAddDirectApparition(this, frame);
    }
    for (auto [observer, mode] : mApparitions) {
        observer->onAddDirectApparition(this, frame);
    }

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        if (isGroup(i)) {
            frame->onMapPointGroupChange(this, i, true);
        }
    }
}

void CML::MapPoint::removeIndirectApparition(PFrame frame) {
#if CML_MAPPOINT_STOREINDIRECTFRAME
    LockGuard lg(mApparitionsMutex);
    uint8_t &mode = mApparitions[frame];
    mode ^= (-(unsigned int) false ^ mode) & (1U << 1);
    if (mode == 0) {
        mApparitions.erase(frame);
    }
#endif
    mIndirectApparitionsNumber--;
    for (auto observer: mObservers) {
        observer->onRemoveIndirectApparition(this, frame);
    }
    for (auto [observer, mode] : mApparitions) {
        observer->onRemoveIndirectApparition(this, frame);
    }
}

void CML::MapPoint::removeDirectApparition(PFrame frame) {
#if CML_MAPPOINT_STOREDIRECTFRAME
    LockGuard lg(mApparitionsMutex);
    uint8_t &mode = mApparitions[frame];
    mode ^= (-(unsigned int) false ^ mode) & (1U << 1);
    if (mode == 0) {
        mApparitions.erase(frame);
    }
#endif
    mDirectApparitionsNumber--;
    for (auto observer: mObservers) {
        observer->onRemoveDirectApparition(this, frame);
    }
    for (auto [observer, mode] : mApparitions) {
        observer->onRemoveDirectApparition(this, frame);
    }
}