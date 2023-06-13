//
// Created by tbelos on 23/07/19.
//

#ifndef CML_DRAWBOARD_H
#define CML_DRAWBOARD_H

#include <vector>
#include <Eigen/Dense>
#include <cml/map/Camera.h>

namespace CML {

    class DrawBoard {

    public:
        virtual void color(Eigen::Vector3f color) = 0;
        virtual void color(float r, float g, float b) = 0;

        virtual void pointSize(int size) = 0;
        virtual void lineWidth(int size) = 0;

        virtual void disableDepthTest() = 0;
        virtual void enableDepthTest() = 0;

        virtual void segment(Eigen::Vector2f a, Eigen::Vector2f b) = 0;
        virtual void point(Eigen::Vector2f a) = 0;

        virtual void segments(std::vector<Eigen::Vector2f> points) = 0;
        virtual void points(std::vector<Eigen::Vector2f> points) = 0;

        virtual void segment(Eigen::Vector3f a, Eigen::Vector3f b) = 0;
        virtual void point(Eigen::Vector3f a) = 0;

        virtual void segments(std::vector<Eigen::Vector3f> points) = 0;
        virtual void points(std::vector<Eigen::Vector3f> points) = 0;

        inline void paintCamera(const Camera &camera) {
            double scale = 0.05;
            double scaled2 = scale / 2;

            WorldPoint center = WorldPoint::fromRelativeCoordinates(Vector3(0,0,0), camera);
            WorldPoint q1 = WorldPoint::fromRelativeCoordinates(Vector3(scaled2,scaled2,scale), camera);
            WorldPoint q2 = WorldPoint::fromRelativeCoordinates(Vector3(scaled2,-scaled2,scale), camera);
            WorldPoint q3 = WorldPoint::fromRelativeCoordinates(Vector3(-scaled2,-scaled2,scale), camera);
            WorldPoint q4 = WorldPoint::fromRelativeCoordinates(Vector3(-scaled2,scaled2,scale), camera);

            segment((Eigen::Vector3f)center.absolute().cast<float>(), q1.absolute().cast<float>());
            segment((Eigen::Vector3f)center.absolute().cast<float>(), q2.absolute().cast<float>());
            segment((Eigen::Vector3f)center.absolute().cast<float>(), q3.absolute().cast<float>());
            segment((Eigen::Vector3f)center.absolute().cast<float>(), q4.absolute().cast<float>());

            segment((Eigen::Vector3f)q1.absolute().cast<float>(), q2.absolute().cast<float>());
            segment((Eigen::Vector3f)q2.absolute().cast<float>(), q3.absolute().cast<float>());
            segment((Eigen::Vector3f)q3.absolute().cast<float>(), q4.absolute().cast<float>());
            segment((Eigen::Vector3f)q4.absolute().cast<float>(), q1.absolute().cast<float>());
        }

        virtual inline void finish() {

        }

    };

}


#endif //CML_DRAWBOARD_H
