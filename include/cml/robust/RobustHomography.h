#ifndef CML_ROBUSTHOMOGRAPHY_H
#define CML_ROBUSTHOMOGRAPHY_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>

namespace CML::Robust {

    class RobustHomography : public AbstractFunction {

    public:
        RobustHomography(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

        scalar_t compute(const List<DistortedVector2d> &distx1, const List<NormalizedVector2d> &x1, const Matrix33 &normalizationMatrix1, const List<DistortedVector2d> &distx2, const List<NormalizedVector2d> &x2, const Matrix33 &normalizationMatrix2, Matrix33 &F);

        CML::List<CML::Camera> reconstruct(const Matrix33 &K);

        std::string getName() final {
            return "Robust Homograpy";
        }

    protected:
        bool makeHypothesis(const List<NormalizedVector2d> &x1, const List<NormalizedVector2d> &x2, Matrix33 &H);

        scalar_t checkHypothesis(const Matrix33 &H21, const Matrix33 &H12, const List<DistortedVector2d> &x1, const List<DistortedVector2d> &x2, List<bool> &inliers);

    private:
        Parameter mIterations = createParameter("Iteration number", 200);
        Parameter mSigma = createParameter("Sigma", 1.0f);

        Matrix33 mH;
        scalar_t mScore = std::numeric_limits<scalar_t>::max();
        List<bool> mInliers;

    };

}

#endif