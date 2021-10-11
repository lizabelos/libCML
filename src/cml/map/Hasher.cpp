//
// Created by tbelos on 19/08/19.
//

#include "cml/map/Frame.h"
#include "cml/map/MapObject.h"

size_t CML::Hasher::operator()(CML::PFrame pFrame) const {
    return pFrame->getId();
}

size_t CML::Hasher::operator()(CML::PPoint pPoint) const {
    return pPoint->getId();
}

bool CML::Comparator::operator() (PPoint pPointA, PPoint pPointB) const {
    return pPointA->getId() > pPointB->getId();
}

bool CML::Comparator::operator() (PFrame pFrameA, PFrame pFrameB) const {
    return pFrameA->getId() > pFrameB->getId();
}