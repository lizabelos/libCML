#ifndef _List_FIELD_CONSENSUS_H
#define _List_FIELD_CONSENSUS_H
// Mismatch removal by List field consensus (VFC)
// Author: Ji Zhao
// Date:   01/25/2015
// Email:  zhaoji84@gmail.com
//
// Parameters:
//   gamma: Percentage of inliers in the samples. This is an inital value
//       for EM iteration, and it is not important. Default value is 0.9.
//   beta: Paramerter of Gaussian Kernel, k(x, y) = exp(-beta* || x - y || ^ 2).
//       Default value is 0.1.
//   lambda: Represents the trade - off between the goodness of data fit
//       and smoothness of the field.Default value is 3.
//   theta: If the posterior probability of a sample being an inlier is
//       larger than theta, then it will be regarded as an inlier.
//       Default value is 0.75.
//   a: Paramerter of the uniform distribution. We assume that the outliers
//       obey a uniform distribution 1 / a. Default Value is 10.
//   MaxIter : Maximum iterition times.Defualt value is 500.
//   ecr: The minimum limitation of the energy change rate in the iteration
//       process. Default value is 1e-5.
//   minP: The posterior probability Matrix P may be singular for matrix
//       inversion.We set the minimum value of P as minP. Default value is
//       1e-5.
//   method: Choose the method for outlier removal.There are three optional
//       methods : NORMAL_VFC, FAST_VFC, SPARSE_VFC. Default value is NORMAL_VFC.
//
//
// Reference
// [1] Jiayi Ma, Ji Zhao, Jinwen Tian, Alan Yuille, and Zhuowen Tu.
//     Robust Point Matching via List Field Consensus, 
//     IEEE Transactions on Image Processing, 23(4), pp. 1706-1721, 2014
// [2] Jiayi Ma, Ji Zhao, Jinwen Tian, Xiang Bai, and Zhuowen Tu.
//     Regularized List Field Learning with Sparse Approximation for Mismatch Removal, 
//     Pattern Recognition, 46(12), pp. 3519-3532, 2013


#include <cml/config.h>
#include <cml/features/cornerTracker/CornerMatcher.h>
#include <iomanip>
#include <iostream>

//#define PRINT_RESULTS
#define DOUBLE_PI (6.283185f)
#define inf (0x3f3f3f3f)
#define NORMAL_VFC 1
#define FAST_VFC   2
#define SPARSE_VFC 3
#define MIN_POINT_NUMBER 5
#define MIN_STANDARD_DEVIATION 0.1

namespace CML::Features {

    class VFC {
    public:
        VFC();

        ~VFC();

        bool setData(List<Vector2f> X1, List<Vector2f> X2);

        inline bool setData(const List<Matching> &matchings) {
            List<Vector2f> X1;
            List<Vector2f> X2;
            X1.resize(matchings.size());
            X2.resize(matchings.size());
            for (size_t i = 0; i < matchings.size(); i++) {
                X1[i] = matchings[i].noAssertGetFeaturePointA().point0().cast<float>();
                X2[i] = matchings[i].noAssertGetFeaturePointB().point0().cast<float>();
            }
            return setData(X1, X2);
        }

        bool normalize();

        Matrixf<Dynamic, Dynamic> constructIntraKernel(List<Vector2f> X);

        Matrixf<Dynamic, Dynamic> constructInterKernel(List<Vector2f> X, List<Vector2f> Y);

        void initialize();

        void getP();

        void calculateTraceCKC();

        void calculateC();

        void calculateV();

        void calculateSigmaSquare();

        void calculateGamma();

        void optimize();

        void optimizeVFC();

        void optimizeFastVFC();

        void optimizeSparseVFC();

        List<int> obtainCorrectMatch();

        inline List<Matching> filterMatchings(const List<Matching> &matchings) {
            List<int> correctMatch = obtainCorrectMatch();
            List<Matching> newMatchings;
            newMatchings.reserve(correctMatch.size());
            for (size_t i = 0; i < correctMatch.size(); i++) {
                newMatchings.emplace_back(matchings[correctMatch[i]]);
            }
            return newMatchings;
        }

        // for FastVFC only
        void calculateTraceCQSQC();

        void calculateCFastVFC();
        // for SparseVFC only
        bool selectSubset();

        void calculateC_SparseVFC();

        inline List<Matching> process(const List<Matching> &matchings) {
            setData(matchings);
            optimize();
            return filterMatchings(matchings);
        }

    private:
        List<int> _matchIdx;
        int _numPt; // number of matches
        int _numDim; // dimensions, fixed as 2 in current version
        int _numElement; // _numPt * _numElement
        List<Vector2f> _lX; // keypoint position in left image
        List<Vector2f> _rX; // keypoint position in right image
        List<Vector2f> _X; // start point of List field
        List<Vector2f> _Y; // end point of List field
        Matrixf<Dynamic, Dynamic> _K; // kernel matrix
        List<Vector2f> _V; // List field
        List<Vector2f> _C; // coefficient
        List<float> _P; // probability
        float _sumP; // sum of _P
        float _sigma2; // deviation of Gaussian noise
        float _E; // energy
        float _traceCKC;

        // parameters
        int _method;
        int _maxIter;
        float _gamma;
        float _beta;
        float _lambda;
        float _theta;
        float _a;
        float _ecr;
        float _minP;

        // parameters for FastVFC
        int _numEig;
        Matrixf<Dynamic, Dynamic> _S; // eigenvalues
        Matrixf<Dynamic, Dynamic> _Q; // eigenLists
        float _traceCQSQC;

        // parameters for SparseVFC
        int _numCtrlPts;
        List<Vector2f> _ctrlPts;
        Matrixf<Dynamic, Dynamic> _U;

        // intermediate variables
        Vector2f _meanLeftX;
        Vector2f _meanRightX;
        float _scaleLeftX;
        float _scaleRightX;
    };

}

#endif
