#ifndef CML_KDTREE_H
#define CML_KDTREE_H

#include "cml/config.h"
#include "cml/map/MapObject.h"
#include <flann/flann.hpp>
#include <flann/algorithms/kdtree_single_index.h>

namespace CML {

#define CML_POINTGRID_WIDTH 64
#define CML_POINTGRID_HEIGHT 64

    template <typename T> class PointGrid {

    public:
        PointGrid(const List<T> &points, const Vector2i &min, const Vector2i &max) : mPoints(points), mMin(min), mMax(max) {
            for (size_t i = 0; i < points.size(); i++) {
                addToGrid(i, gridPos(points[i]));
            }
        }

        template <typename U> List<NearestNeighbor> searchInRadius(const U &point, scalar_t radius) {
            Vector2i center = gridPos(point);
            Vector2i elipseRadius = Vector2i(1 + radius * CML_POINTGRID_WIDTH  / (mMax.x() - mMin.x()), 1 + radius * CML_POINTGRID_HEIGHT / (mMax.y() - mMin.y()));
            Vector2i topLeft = center - elipseRadius;
            Vector2i bottomRight = center + elipseRadius;

            List<NearestNeighbor> result;

            for (int x = topLeft.x(); x <= bottomRight.x(); x++) {
                if (x < 0 || x >= CML_POINTGRID_WIDTH) {
                    continue;
                }
                for (int y = topLeft.y(); y <= bottomRight.y(); y++) {
                    if (y < 0 || y >= CML_POINTGRID_HEIGHT) {
                        continue;
                    }
                    for (auto index : mIndex[x][y]) {

                        scalar_t d = (Vector2(point.x(), point.y()) - Vector2(mPoints[index].x(), mPoints[index].y())).norm();
                        if (d <= radius) {
                            result.emplace_back(index, d);
                        }

                    }
                }
            }

            return result;

        }

    protected:
        template <typename U> Vector2i gridPos(const U &point) {
            return {
                    (point.x() - mMin.x()) * CML_POINTGRID_WIDTH  / (mMax.x() - mMin.x()),
                    (point.y() - mMin.y()) * CML_POINTGRID_HEIGHT / (mMax.y() - mMin.y())
            };
        }

        void addToGrid(size_t index, const Vector2i &pos) {
            if (pos.x() < 0 || pos.y() < 0 || pos.x() >= CML_POINTGRID_WIDTH || pos.y() >= CML_POINTGRID_HEIGHT) {
                return;
            }
            mIndex[pos.x()][pos.y()].emplace_back(index);
        }

    private:
        List<T> mPoints;
        Vector2i mMin, mMax;
        List<size_t> mIndex[CML_POINTGRID_WIDTH][CML_POINTGRID_HEIGHT];
    };

    class PointKDTree {

    public:
        template <typename T> explicit PointKDTree(List<T> features) {
            assertThrow(features.size() > 0, "Can't build a KDTree with no point");

            mTargetData = new int[features.size() * 2];
            for (size_t i = 0; i < features.size(); i++) {
                mTargetData[i * 2 + 0] = (int)features[i].x();
                mTargetData[i * 2 + 1] = (int)features[i].y();
            }

            mDataset = new flann::Matrix<int>(mTargetData, features.size(), 2);
            mKDTree = new flann::KDTreeSingleIndex<flann::L2_Simple<int>>(*mDataset, flann::KDTreeIndexParams(1));

            mKDTree->buildIndex();
        }

        explicit PointKDTree(List<Corner> features, int level) {
            assertThrow(features.size() > 0, "Can't build a KDTree with no point");

            mTargetData = new int[features.size() * 2];
            for (size_t i = 0; i < features.size(); i++) {
                mTargetData[i * 2 + 0] = (int)features[i].point(level).x();
                mTargetData[i * 2 + 1] = (int)features[i].point(level).y();
            }

            mDataset = new flann::Matrix<int>(mTargetData, features.size(), 2);
            mKDTree = new flann::KDTreeSingleIndex<flann::L2_Simple<int>>(*mDataset, flann::KDTreeIndexParams(1));

            mKDTree->buildIndex();
        }

        explicit PointKDTree(List<PPoint> features, int level) {
            assertThrow(features.size() > 0, "Can't build a KDTree with no point");

            mTargetData = new int[features.size() * 2];
            for (size_t i = 0; i < features.size(); i++) {
                mTargetData[i * 2 + 0] = (int)features[i]->getReferenceCorner().point(level).x();
                mTargetData[i * 2 + 1] = (int)features[i]->getReferenceCorner().point(level).y();
            }

            mDataset = new flann::Matrix<int>(mTargetData, features.size(), 2);
            mKDTree = new flann::KDTreeSingleIndex<flann::L2_Simple<int>>(*mDataset, flann::KDTreeIndexParams(1));

            mKDTree->buildIndex();
        }

        ~PointKDTree() {
            delete mKDTree;
            delete mDataset;
            delete[] mTargetData;
        }

        template <typename T> CML::List<NearestNeighbor> getNearestNeighbors(T point, int n) {

            int position[2];
            position[0] = point.x();
            position[1] = point.y();

            int results[n];
            float distances[n];

            flann::Matrix<int> query(position, 1, 2);
            flann::Matrix<int> indices(results, 1, n);
            flann::Matrix<float> dists(distances, 1, n);
            int count = mKDTree->knnSearch(query, indices, dists, n, flann::SearchParams(-1));

            List<NearestNeighbor> nearestNeighbor;
            nearestNeighbor.resize(count);

            for (int i = 0; i < count; i++) {

                nearestNeighbor[i].index = indices[0][i];
                nearestNeighbor[i].distance = dists[0][i];

            }

            return nearestNeighbor;

        }

        template <typename T> CML::List<NearestNeighbor> getNearestNeighborsInRadius(T point, float distance) {

            int position[2];
            position[0] = point.x();
            position[1] = point.y();

            int results[1000];
            float distances[1000];

            flann::Matrix<int> query(position, 1, 2);
            flann::Matrix<int> indices(results, 1, 1000);
            flann::Matrix<float> dists(distances, 1, 1000);
            int count = mKDTree->radiusSearch(query, indices, dists, distance, flann::SearchParams(-1));

            List<NearestNeighbor> nearestNeighbor;
            nearestNeighbor.resize(count);

            for (int i = 0; i < count; i++) {

                nearestNeighbor[i].index = indices[0][i];
                nearestNeighbor[i].distance = dists[0][i];

            }

            return nearestNeighbor;


        }

    private:
        int *mTargetData;
        flann::Matrix<int> *mDataset;
        flann::KDTreeSingleIndex<flann::L2_Simple<int>> *mKDTree;

    };

    class LSHTree {

    public:
        explicit LSHTree(unsigned char *descriptors, int descriptorNumber, int descriptorSize) : mDescriptorSize(descriptorSize) {
            assertThrow(descriptorNumber > 0, "Can't build a LSHTree with no descriptor");
            assertThrow(descriptorSize > 0, "Can't build a LSHTree with empty descriptor");

            flann::Matrix<unsigned char> matTrain(descriptors, descriptorNumber, descriptorSize);
            mTree = new flann::HierarchicalClusteringIndex<flann::Hamming<unsigned char>>(matTrain, flann::HierarchicalClusteringIndexParams());
            mTree->buildIndex();
        }

        ~LSHTree() {
            delete mTree;
        }

        CML::List<NearestNeighbor> getNearestNeighbors(const unsigned char *descriptor, int n) {

            flann::Matrix<unsigned char> query((unsigned char*)descriptor, 1, mDescriptorSize); // This is bad to do this. Find an other way to put the const...
            flann::SearchParams searchParam;

            int results[n];
            unsigned int distances[n];

            flann::Matrix<int> indices(results, 1, n);
            flann::Matrix<unsigned int> dists(distances, 1, n);
            int count = mTree->knnSearch(query, indices, dists, n, searchParam);

            List<NearestNeighbor> nearestNeighbor;
            nearestNeighbor.resize(count);

            for (int i = 0; i < count; i++) {

                nearestNeighbor[i].index = indices[0][i];
                nearestNeighbor[i].distance = dists[0][i];

            }

            return nearestNeighbor;
        }

    private:
        flann::HierarchicalClusteringIndex<flann::Hamming<unsigned char>> *mTree;
        int mDescriptorSize;

    };


}

#endif