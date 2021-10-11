//
// Created by thomas on 19/06/2020.
//

#ifndef CML_ALL_DISTANCEMAP_H
#define CML_ALL_DISTANCEMAP_H

#include <cml/config.h>
#include <cml/image/Array2D.h>

#include <queue>

namespace CML {

    class DistanceMap {

    public:
        EIGEN_STRONG_INLINE DistanceMap(int width, int height) : mDistanceArray(width, height, 9999), mMaxDistance(9999) {

        }

        EIGEN_STRONG_INLINE DistanceMap(int width, int height, int maxDist) : mDistanceArray(width, height, maxDist), mMaxDistance(maxDist) {

        }

        template <typename T> void addPoints(T points) {
            for (auto point : points) {
                _add(point.x(), point.y());
            }
            grow();
        }

        template <typename T> void addPoint(T point) {
            _add(point.x(), point.y());
            grow();
        }

        template <typename T> int get(T point) {
            return mDistanceArray(point.x(), point.y());
        }

        EIGEN_STRONG_INLINE int get(int x, int y) {
            return mDistanceArray(x, y);
        }

    protected:
        EIGEN_STRONG_INLINE void _add(int x, int y) {
            int d = mMaxDistance + 3;
            if (x > d && x < mDistanceArray.getWidth() - d && y > d && y < mDistanceArray.getHeight() - d) {
                fast_queue(x, y, 0);
            } else {
                queue(x, y, 0);
            }
        }

        EIGEN_STRONG_INLINE void queueNeighbors(int x, int y, int dist) {
            queue(x - 1, y, dist + 1);
            queue(x + 1, y, dist + 1);
            queue(x, y - 1, dist + 1);
            queue(x, y + 1, dist + 1);

            queue(x - 1, y - 1, dist + 1);
            queue(x + 1, y + 1, dist + 1);
            queue(x + 1, y - 1, dist + 1);
            queue(x - 1, y + 1, dist + 1);
        }

        EIGEN_STRONG_INLINE void queue(int x, int y, int dist) {
            if (x < 0) {
                return;
            }
            if (y < 0) {
                return;
            }
            if (x >= mDistanceArray.getWidth()) {
                return;
            }
            if (y >= mDistanceArray.getHeight()) {
                return;
            }
            if (dist < mDistanceArray(x, y)) {
                mDistanceArray(x, y) = dist;
                mQueue.emplace(Pair<Vector2i, int>(Vector2i(x, y), dist));
            }
        }

        EIGEN_STRONG_INLINE void fast_queueNeighbors(int x, int y, int dist) {
            queue(x - 1, y, dist + 1);
            queue(x + 1, y, dist + 1);
            queue(x, y - 1, dist + 1);
            queue(x, y + 1, dist + 1);

            queue(x - 1, y - 1, dist + 1);
            queue(x + 1, y + 1, dist + 1);
            queue(x + 1, y - 1, dist + 1);
            queue(x - 1, y + 1, dist + 1);
        }

        EIGEN_STRONG_INLINE void fast_queue(int x, int y, int dist) {
            if (dist < mDistanceArray(x, y)) {
                mDistanceArray(x, y) = dist;
                mQueue.emplace(Pair<Vector2i, int>(Vector2i(x, y), dist));
            }
        }

        EIGEN_STRONG_INLINE void grow() {
            while (!mQueue.empty()) {
                Pair<Vector2i, int> pair = mQueue.front();
                mQueue.pop();
                queueNeighbors(pair.first.x(), pair.first.y(), pair.second);
            }
        }

    private:
        Array2D<int> mDistanceArray;
        std::queue<Pair<Vector2i, int>> mQueue;
        int mMaxDistance;
    };

}

#endif //CML_ALL_DISTANCEMAP_H
