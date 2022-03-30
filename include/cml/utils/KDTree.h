#ifndef CML_KDTREE_H
#define CML_KDTREE_H

#include "cml/config.h"
#include "cml/map/MapObject.h"

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

        template <typename U> void searchInRadius(const U &point, scalar_t radius, List<NearestNeighbor> &result) {
            Vector2i center = gridPos(point);
            Vector2i elipseRadius = Vector2i(1 + radius * CML_POINTGRID_WIDTH  / (mMax.x() - mMin.x()), 1 + radius * CML_POINTGRID_HEIGHT / (mMax.y() - mMin.y()));
            Vector2i topLeft = center - elipseRadius;
            Vector2i bottomRight = center + elipseRadius;

            result.clear();

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


        }

        template <typename U> void searchInRadiusNum(const U &point, size_t num, List<NearestNeighbor> &result) {

            result.clear();
            result.reserve(num * 2);
            Vector2i center = gridPos(point);
            size_t radius = 0;
            Vector2i topLeft;
            Vector2i bottomRight;

            while (radius < CML_POINTGRID_WIDTH) {

                topLeft = center - Vector2i(radius, radius);
                bottomRight = center + Vector2i(radius, radius);
                size_t n = 0;

                for (int x = topLeft.x(); x <= bottomRight.x(); x++) {
                    if (x < 0 || x >= CML_POINTGRID_WIDTH) {
                        continue;
                    }
                    for (int y = topLeft.y(); y <= bottomRight.y(); y++) {
                        if (y < 0 || y >= CML_POINTGRID_HEIGHT) {
                            continue;
                        }
                        n = n + mIndex[x][y].size();
                    }
                }

                if (n >= num) {
                    break;
                }

                radius++;

            }

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
                        result.emplace_back(index, d);
                    }
                }
            }

            std::sort(result.begin(), result.end(),
                 [](const NearestNeighbor & a, const NearestNeighbor & b) -> bool
                 {
                     return a.distance < b.distance;
                 });

            if (result.size() >= 2) {
                assertThrow(result[0].distance <= result[result.size() - 1].distance, "Invalid order");
            }

            if (result.size() > num) {
                result.resize(num);
            }

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


}

#endif