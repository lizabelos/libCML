#include "cml/robust/RobustFundamental8Points.h"
#include "cml/robust/backend/Ransac.h"

CML::scalar_t CML::Robust::RobustFundamental8Points::compute(
                                                         const List<DistortedVector2d> &distx1,
                                                         const List<NormalizedVector2d> &x1,
                                                         const Matrix33 &normalizationMatrix1,
                                                         const List<DistortedVector2d> &distx2,
                                                         const List<NormalizedVector2d> &x2,
                                                         const Matrix33 &normalizationMatrix2,
                                                         Matrix33 &F) {

    List<int> indexes;
    indexes.resize(x1.size());
    for (size_t i = 0; i < x1.size(); i++) {
        indexes[i] = i;
    }

    mScore = 0;

    for (int i = 0; i < mIterations.i(); i++) {
        List<int> subsampleIndexes;
        processSubsample(indexes, subsampleIndexes, 8);

        List<NormalizedVector2d> x1subsample, x2subsample;
        x1subsample.resize(8); x2subsample.resize(8);

        for (int j = 0; j < 8; j++) {
            x1subsample[j] = x1[indexes[subsampleIndexes[j]]];
            x2subsample[j] = x2[indexes[subsampleIndexes[j]]];
        }

        Matrix33 currentF;
        bool success = makeHypothesis(x1subsample, x2subsample, currentF);

        if (!success) {
            continue;
        }

        currentF = normalizationMatrix2.transpose() * currentF * normalizationMatrix1;

        List<bool> currentInliers;
        scalar_t currentScore = checkHypothesis(currentF, distx1, distx2, currentInliers);

        if (currentScore > mScore) {
            mInliers = currentInliers;
            mScore = currentScore;
            mF = currentF;
        }

    }

    F = mF;
    return mScore;

}

CML::List<CML::Camera> CML::Robust::RobustFundamental8Points::reconstruct(const Matrix33 &K) {

    // Compute essential matrix
    Matrix33 E = K.transpose() * mF * K;

#if 0
    // Decompose the essential matrix
    Eigen::JacobiSVD<Matrix33> svd = Eigen::JacobiSVD<Matrix33>(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix33 U = svd.matrixU();
    Matrix33 V = svd.matrixV();

    Matrix33 W = Matrix33::Zero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;

    Matrix33 R1 = U * W * V.transpose();
    Matrix33 R2 = U * W.transpose() * V.transpose();

    Vector3 t1 = U.col(2).normalized();
    Vector3 t2 = -U.col(2).normalized();

    CML::List<Camera> results;
    results.emplace_back(Camera(t1, R1));
    results.emplace_back(Camera(t1, R2));
    results.emplace_back(Camera(t2, R1));
    results.emplace_back(Camera(t2, R2));

    return results;

#else

    // Horn method from https://github.com/MasteringOpenCV/code/blob/master/Chapter4_StructureFromMotion/FindCameraMatrices.cpp

    Matrix33 EEt = E * E.transpose();
    Vector3 e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(2));
    Vector3 b1,b2;

    //Method 1
    Matrix33 bbt = 0.5 * EEt.trace() * Matrix33::Identity() - EEt; //Horn90 (12)
    Vector3 bbt_diag = bbt.diagonal();
    if (bbt_diag(0) > bbt_diag(1) && bbt_diag(0) > bbt_diag(2)) {
        b1 = bbt.row(0) / sqrt(bbt_diag(0));
        b2 = -b1;
    } else if (bbt_diag(1) > bbt_diag(0) && bbt_diag(1) > bbt_diag(2)) {
        b1 = bbt.row(1) / sqrt(bbt_diag(1));
        b2 = -b1;
    } else {
        b1 = bbt.row(2) / sqrt(bbt_diag(2));
        b2 = -b1;
    }

    //Horn90 (19)
    Matrix33 cofactors; cofactors.col(0) = e1e2; cofactors.col(1) = e2e0; cofactors.col(2) = e0e1;
    cofactors.transposeInPlace();

    //B = [b]_x , see Horn90 (6) and http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
    Matrix33 B1; B1 <<	0,-b1(2),b1(1),
            b1(2),0,-b1(0),
            -b1(1),b1(0),0;
    Matrix33 B2; B2 <<	0,-b2(2),b2(1),
            b2(2),0,-b2(0),
            -b2(1),b2(0),0;


    //Horn90 (24)
    Matrix33 R1 = (cofactors.transpose() - B1*E) / b1.dot(b1);
    Matrix33 R2 = (cofactors.transpose() - B2*E) / b2.dot(b2);
    Vector3 t1 = b1.normalized();
    Vector3 t2 = b2.normalized();

    CML::List<Camera> results;
    results.emplace_back(Camera(t1, R1));
    results.emplace_back(Camera(t1, R2));
    results.emplace_back(Camera(t2, R1));
    results.emplace_back(Camera(t2, R2));

    return results;

#endif
}

bool
CML::Robust::RobustFundamental8Points::makeHypothesis(const List<CML::NormalizedVector2d> &x1,
                                                           const List<CML::NormalizedVector2d> &x2,
                                                           Matrix33 &F) {


    Matrix<8, 9> A;

    for(int i=0; i<8; i++)
    {
        const scalar_t u1 = x1[i](0);
        const scalar_t v1 = x1[i](1);
        const scalar_t u2 = x2[i](0);
        const scalar_t v2 = x2[i](1);

        A(i,0) = u2 * u1;
        A(i,1) = u2 * v1;
        A(i,2) = u2 * 1;
        A(i,3) = v2 * u1;
        A(i,4) = v2 * v1;
        A(i,5) = v2 * 1;
        A(i,6) = 1  * u1;
        A(i,7) = 1  * v1;
        A(i,8) = 1  * 1;
    }


    Eigen::JacobiSVD<Matrix<8, 9>> preSvd(A, Eigen::ComputeFullV);

    Vector9 FpreRow = preSvd.matrixV().transpose().row(8);
    Matrix33 Fpre;
    Fpre << FpreRow(0), FpreRow(1), FpreRow(2),
            FpreRow(3), FpreRow(4), FpreRow(5),
            FpreRow(6), FpreRow(7), FpreRow(8);

    // Enforce rank 2 of the fundamental matrix
    Eigen::JacobiSVD<Matrix33> svd(Fpre, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vector3 dPrime(svd.singularValues()(0), svd.singularValues()(1), 0.0);
    F = svd.matrixU() * dPrime.asDiagonal() * svd.matrixV().transpose();
    return true;

}

CML::scalar_t CML::Robust::RobustFundamental8Points::checkHypothesis(const Matrix33 &F,
                                                                   const List<CML::DistortedVector2d> &x1,
                                                                   const List<CML::DistortedVector2d> &x2,
                                                                   List<bool> &inliers) {
    // Code from ORB SLAM 2

    const int N = x1.size();

    const scalar_t f11 = F(0,0);
    const scalar_t f12 = F(0,1);
    const scalar_t f13 = F(0,2);
    const scalar_t f21 = F(1,0);
    const scalar_t f22 = F(1,1);
    const scalar_t f23 = F(1,2);
    const scalar_t f31 = F(2,0);
    const scalar_t f32 = F(2,1);
    const scalar_t f33 = F(2,2);

    inliers.resize(N);

    scalar_t score = 0;

    const scalar_t th = 3.841;
    const scalar_t thScore = 5.991;

    const scalar_t invSigmaSquare = 1.0 / (mSigma.f() * mSigma.f());

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const scalar_t u1 = x1[i].x();
        const scalar_t v1 = x1[i].y();
        const scalar_t u2 = x2[i].x();
        const scalar_t v2 = x2[i].y();

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const scalar_t a2 = f11*u1+f12*v1+f13;
        const scalar_t b2 = f21*u1+f22*v1+f23;
        const scalar_t c2 = f31*u1+f32*v1+f33;

        const scalar_t num2 = a2*u2+b2*v2+c2;

        const scalar_t squareDist1 = num2*num2/(a2*a2+b2*b2);

        const scalar_t chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th) {
            bIn = false;
        }
        else {
            score += thScore - chiSquare1;
        }

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const scalar_t a1 = f11*u2+f21*v2+f31;
        const scalar_t b1 = f12*u2+f22*v2+f32;
        const scalar_t c1 = f13*u2+f23*v2+f33;

        const scalar_t num1 = a1*u1+b1*v1+c1;

        const scalar_t squareDist2 = num1*num1/(a1*a1+b1*b1);

        const scalar_t chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th) {
            bIn = false;
        }
        else {
            score += thScore - chiSquare2;
        }

        inliers[i] = bIn;
    }

    return score;
}
