//
// Created by tbelos on 19/04/19.
//

#include "cml/map/MapObject.h"

CML::PoolAllocator CML::MapPoint::allocator{16384};

void CML::MapPoint::recreate(size_t id, PFrame reference, FeatureIndex referenceIndex, MapPointType type, scalar_t *coordinate, scalar_t *color, scalar_t *uncertainty, unsigned int *groups)
{
    signalMethodStart("MapPoint::recreate");

    mPrivate.reset();

    mId = id;
    mReference = reference;
    mReferenceFeatureIndex = referenceIndex;
    mWorldCoordinate = coordinate;
    mUncertainty = uncertainty;
    mColor = color;
    mGroups = groups;

    assertThrow(reference != nullptr, "with a null reference pointer");

    mHash = integerHashing(mId);

//    assertThrow(reference->isInside(corner.point(0), 0, 0), "corner is not inside the reference");

    mOptions = 0;
    mOptions ^= (-((int)type) ^ mOptions) & ((uint8_t)1 << 0);

#if CML_MAPPOINT_STORECORNER
    mReferenceCorner = reference->getFeaturePoint(mReferenceFeatureIndex);
#endif


    // reference->getCamera().subscribeObserver(this);

    switch (type) {
        case DIRECTTYPE:
            break;
        case INDIRECTTYPE:
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

void CML::MapPoint::destroy() {
    signalMethodStart("MapPoint::destroy");
    auto apparitionsDirect = getDirectApparitions();
    auto indirectApparitions = getIndirectApparitions();
    {
        signalMethodStart("MapPoint::destroy::removeDirectApparition");
        for (auto frame: apparitionsDirect) {
            for (auto observer: apparitionsDirect) {
                observer->onRemoveDirectApparition(this, frame);
            }
            for (auto observer: indirectApparitions) {
                observer->onRemoveDirectApparition(this, frame);
            }
            for (auto observer: mObservers) {
                observer->onRemoveDirectApparition(this, frame);
            }
        }
    }
    {
        signalMethodStart("MapPoint::destroy::removeIndirectApparition");
        for (auto frame: indirectApparitions) {
            for (auto observer: apparitionsDirect) {
                observer->onRemoveIndirectApparition(this, frame);
            }
            for (auto observer: indirectApparitions) {
                observer->onRemoveIndirectApparition(this, frame);
            }
            for (auto observer: mObservers) {
                observer->onRemoveIndirectApparition(this, frame);
            }
        }
    }
    for (auto observer : mObservers) {
        signalMethodStart("MapPoint::destroy::removeObserver");
        observer->onMapPointDestroyed(this);
    }
    {
        signalMethodStart("MapPoint::destroy::clear");
        freeDescriptor();
        mApparitionsIndirect.clear();
        mApparitionsDirect.clear();
        mObservers.clear();
    }
}

CML::scalar_t CML::MapPoint::getReferenceInverseDepth() const {
    if (isReferenceDepth()) {
        return mReferenceInverseDepth;
    } else {
        return 1.0 / getWorldCoordinate().relative(mReference->getCamera()).z();
    }
}

void CML::MapPoint::setReferenceInverseDepth(scalar_t idepth) {
    assertDeterministic("Set reference inverse depth", idepth);
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
    assertDeterministic("Set world coordinate", worldCoordinate.absolute().sum());
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
    signalMethodStart("MapPoint::addIndirectApparition");
//    assertThrow(mIndirectApparitions.count(frame) == 0, "Already added the frame");
#if CML_MAPPOINT_STOREINDIRECTFRAME
    {
        LockGuard lg(mApparitionsMutex);
        mApparitionsIndirect.insert(frame);
    }
#endif
    if (mLastSeen.isNull()) mLastSeen = frame;
    else if (mLastSeen->getId() < frame->getId()) mLastSeen = frame;
    // mIndirectApparitions[frame] = where;

    mIndirectApparitionsNumber++;

    for (auto observer: mObservers) {
        observer->onAddIndirectApparition(this, frame);
    }
    Set<OptPFrame> observers;
    observers.insert(mApparitionsIndirect.begin(), mApparitionsIndirect.end());
    observers.insert(mApparitionsDirect.begin(), mApparitionsDirect.end());
    for (auto observer : observers) {
        observer->onAddIndirectApparition(this, frame);
    }

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        if (isGroup(i)) {
            frame->onMapPointGroupChange(this, i, true);
        }
    }


}

void CML::MapPoint::addDirectApparition(PFrame frame) {
    signalMethodStart("MapPoint::addDirectApparition");
#if CML_MAPPOINT_STOREDIRECTFRAME
    {
        LockGuard lg(mApparitionsMutex);
        mApparitionsDirect.insert(frame);
    }
#endif

    mDirectApparitionsNumber++;

    for (auto observer: mObservers) {
        observer->onAddDirectApparition(this, frame);
    }

    for (auto observer : mApparitionsIndirect) {
        observer->onAddDirectApparition(this, frame);
        observer->setFlagged(true);
    }

    for (auto observer : mApparitionsDirect) {
        if (!observer->isFlagged()) {
            observer->onAddDirectApparition(this, frame);
        }
    }

    for (auto observer : mApparitionsIndirect) {
        observer->setFlagged(false);
    }

    for (int i = 0; i < MAPOBJECT_GROUP_MAXSIZE; i++) {
        if (isGroup(i)) {
            frame->onMapPointGroupChange(this, i, true);
        }
    }

}

void CML::MapPoint::removeIndirectApparition(PFrame frame) {
    signalMethodStart("MapPoint::removeIndirectApparition");
#if CML_MAPPOINT_STOREINDIRECTFRAME
    LockGuard lg(mApparitionsMutex);
    mApparitionsIndirect.erase(frame);
#endif
    mIndirectApparitionsNumber--;
    for (auto observer: mObservers) {
        observer->onRemoveIndirectApparition(this, frame);
    }
    Set<OptPFrame> observers;
    observers.insert(mApparitionsIndirect.begin(), mApparitionsIndirect.end());
    observers.insert(mApparitionsDirect.begin(), mApparitionsDirect.end());
    for (auto observer : observers) {
        observer->onRemoveIndirectApparition(this, frame);
    }

}

void CML::MapPoint::removeDirectApparition(PFrame frame) {
    signalMethodStart("MapPoint::removeDirectApparition");
#if CML_MAPPOINT_STOREDIRECTFRAME
    LockGuard lg(mApparitionsMutex);
    mApparitionsDirect.erase(frame);
#endif
    mDirectApparitionsNumber--;
    for (auto observer: mObservers) {
        observer->onRemoveDirectApparition(this, frame);
    }
    Set<OptPFrame> observers;
    observers.insert(mApparitionsIndirect.begin(), mApparitionsIndirect.end());
    observers.insert(mApparitionsDirect.begin(), mApparitionsDirect.end());
    for (auto observer : observers) {
        observer->onRemoveDirectApparition(this, frame);
    }

}