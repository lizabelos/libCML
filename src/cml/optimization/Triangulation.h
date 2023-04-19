//
// Created by belos on 15/08/2019.
//

#ifndef CML_TRIANGULATION_H
#define CML_TRIANGULATION_H

#include "cml/config.h"
#include "cml/map/Camera.h"
#include "cml/features/cornerTracker/CornerMatcher.h"

namespace CML {

    class Triangulator : public AbstractFunction {

    public:
        Triangulator(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

        virtual Vector3 triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &pointA, const UndistortedVector2d &pointB) = 0;

        inline List<Vector3> triangulate(const Camera &cameraA, const Camera &cameraB, const List<UndistortedVector2d> &pointsA, const List<UndistortedVector2d> &pointsB, bool parallel = true) {
            List<Vector3> results;
            results.resize(pointsA.size());

            if (parallel) {
                #if CML_USE_OPENMP
                #pragma omp  for schedule(dynamic)
                #endif
                for (size_t i = 0; i < pointsA.size(); i++) {
                    results[i] = triangulate(cameraA, cameraB, pointsA[i], pointsB[i]);
                }
            } else {
                for (size_t i = 0; i < pointsA.size(); i++) {
                    results[i] = triangulate(cameraA, cameraB, pointsA[i], pointsB[i]);
                }
            }

            return results;
        }

        virtual inline Vector3 triangulate(PFrame frameA, PFrame frameB, const Matching &matching) {
            return triangulate(frameA->getCamera(), frameB->getCamera(), matching.getUndistortedA(frameA, 0), matching.getUndistortedB(frameB, 0));
        }

        virtual inline List<Vector3> triangulate(PFrame frameA, PFrame frameB, const List<Matching> &matchings) {
            List<Vector3> results;
            results.resize(matchings.size());

            #if CML_USE_OPENMP
            #pragma omp  for schedule(dynamic)
            #endif
            for (size_t i = 0; i < matchings.size(); i++) {
                results[i] = triangulate(frameA, frameB, matchings[i]);
            }

            return results;
        }

        virtual inline List<Vector3> triangulate(PFrame frameA, PFrame frameB, const List<Matching> &matchings, List<scalar_t> &parallax, bool parallel = true) {
            List<Vector3> results;
            results.resize(matchings.size());
            parallax.resize(matchings.size());

            if (parallel) {
                #if CML_USE_OPENMP
                #pragma omp  for schedule(dynamic)
                #endif
                for (size_t i = 0; i < matchings.size(); i++) {
                    results[i] = triangulate(frameA, frameB, matchings[i]);
                    parallax[i] = frameA->getCamera().parallax(frameB->getCamera(), results[i]);
                }
            } else {
                for (size_t i = 0; i < matchings.size(); i++) {
                    results[i] = triangulate(frameA, frameB, matchings[i]);
                    parallax[i] = frameA->getCamera().parallax(frameB->getCamera(), results[i]);
                }
            }

            return results;
        }

        virtual Vector3 triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &pointA, const UndistortedVector2d &pointB, scalar_t &parallax) {

            Vector3 result = triangulate(cameraA, cameraB, pointA, pointB);
            parallax = cameraA.parallax(cameraB, result);
            return result;

        }

        virtual std::string getMethodName() = 0;

        std::string getName() override {
            return getMethodName();
        }

    };

    
    /**
     * @article{hartley1997triangulation,
     *   title={Triangulation},
     *   author={Hartley, Richard I and Sturm, Peter},
     *   journal={Computer vision and image understanding},
     *   volume={68},
     *   number={2},
     *   pages={146--157},
     *   year={1997},
     *   publisher={Elsevier}
     * }
     *
     * Source-Code:    https://github.com/MasteringOpenCV/code
     * Book:           http://www.packtpub.com/cool-projects-with-opencv/book
     * Copyright:      Packt Publishing 2012.
     *
     */
    class Hartley1997LinearTriangulation : public Triangulator {

    public:
        Hartley1997LinearTriangulation(Ptr<AbstractFunction, NonNullable> parent) : Triangulator(parent) {};

        Vector3 triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &pointA, const UndistortedVector2d &pointB) final;

        std::string getMethodName() final {
            return "Hartley 1997 Linear Triangulation";
        }

    };


    /**
     * @book{hartley2003multiple,
     *   title={Multiple view geometry in computer vision},
     *   author={Hartley, Richard and Zisserman, Andrew},
     *   year={2003},
     *   publisher={Cambridge university press}
     * }
     *
     * See page 312
     *
     */
    class Hartley2003Triangulation : public Triangulator {

    public:
        Hartley2003Triangulation(Ptr<AbstractFunction, NonNullable> parent) : Triangulator(parent) {};

        Vector3 triangulateNCam(const List<Camera> &cameras, const List<UndistortedVector2d> &undistortedVectors);

        void triangulateNCam(PPoint mapPoint, List<PFrame> frames);

        Vector3 triangulate(const Camera &cameraA, const Camera &cameraB, const UndistortedVector2d &xA, const UndistortedVector2d &xB) final;

        std::string getMethodName() final {
            return "Hartley 2003";
        }
    };

}


#endif //CML_TRIANGULATION_H
