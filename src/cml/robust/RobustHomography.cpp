#include "cml/robust/RobustHomography.h"
#include "cml/robust/backend/Ransac.h"

CML::scalar_t CML::Robust::RobustHomography::compute(
                                                const List<DistortedVector2d> &distx1,
                                                const List<NormalizedVector2d> &x1,
                                                const Matrix33 &normalizationMatrix1,
                                                const List<DistortedVector2d> &distx2,
                                                const List<NormalizedVector2d> &x2,
                                                const Matrix33 &normalizationMatrix2,
                                                Matrix33 &H) {


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

        Matrix33 currentH21;
        bool success = makeHypothesis(x1subsample, x2subsample, currentH21);

        if (!success) {
            continue;
        }

        currentH21 = normalizationMatrix2.inverse() * currentH21 * normalizationMatrix1;
        Matrix33 currentH12 = currentH21.inverse();

        List<bool> currentInliers;
        scalar_t currentScore = checkHypothesis(currentH21, currentH12, distx1, distx2, currentInliers);

        if (currentScore > mScore) {
            mInliers = currentInliers;
            mScore = currentScore;
            mH = currentH21;
        }

    }

    H = mH;
    return mScore;

}

CML::List<CML::Camera> CML::Robust::RobustHomography::reconstruct(const Matrix33 &K) {
    // From ORB SLAM 2

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    Matrix33 A = K.inverse() * mH * K;

    Eigen::JacobiSVD<Matrix33> svd = Eigen::JacobiSVD<Matrix33>(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Matrix33 U = svd.matrixU();
    Matrix33 V = svd.matrixV();
    Matrix33 Vt = V.transpose();

    scalar_t s = U.determinant() * Vt.determinant();

    auto w = svd.singularValues();

    scalar_t d1 = w(0);
    scalar_t d2 = w(1);
    scalar_t d3 = w(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return List<Camera>();
    }

    List<Camera> results;

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    scalar_t aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    scalar_t aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    scalar_t x1[] = {aux1,aux1,-aux1,-aux1};
    scalar_t x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    scalar_t aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    scalar_t ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    scalar_t stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        Matrix33 Rp = Matrix33::Identity();
        Rp(0,0)=ctheta;
        Rp(0,2)=-stheta[i];
        Rp(2,0)=stheta[i];
        Rp(2,2)=ctheta;

        Matrix33 R = s*U*Rp*Vt;
        // vR.push_back(R);

        Vector3 tp;
        tp(0)=x1[i];
        tp(1)=0;
        tp(2)=-x3[i];
        tp*=d1-d3;

        Vector3 t = U*tp;
        t.normalize();

        results.emplace_back(Camera(t, R));
    }

    //case d'=-d2
    scalar_t aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    scalar_t cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    scalar_t sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        Matrix33 Rp = Matrix33::Identity();
        Rp(0,0)=cphi;
        Rp(0,2)=sphi[i];
        Rp(1,1)=-1;
        Rp(2,0)=sphi[i];
        Rp(2,2)=-cphi;

        Matrix33 R = s*U*Rp*Vt;

        Vector3 tp;
        tp(0)=x1[i];
        tp(1)=0;
        tp(2)=x3[i];
        tp*=d1+d3;

        Vector3 t = U*tp;
        t.normalize();

        results.emplace_back(Camera(t, R));
    }

    return results;
}

bool CML::Robust::RobustHomography::makeHypothesis(const List<NormalizedVector2d> &x1,
                                                        const List<NormalizedVector2d> &x2,
                                                        Matrix33 &H) {

    Matrix<16, 9> A;
    for (int i = 0; i < 8; i++) {

        A(2*i,0) = 0.0;
        A(2*i,1) = 0.0;
        A(2*i,2) = 0.0;
        A(2*i,3) = -x1[i].x();
        A(2*i,4) = -x1[i].y();
        A(2*i,5) = -1.0;
        A(2*i,6) = x2[i].y() * x1[i].x();
        A(2*i,7) = x2[i].y() * x1[i].y();
        A(2*i,8) = x2[i].y();

        A(2*i+1,0) = x1[i].x();
        A(2*i+1,1) = x1[i].y();
        A(2*i+1,2) = 1.0;
        A(2*i+1,3) = 0.0;
        A(2*i+1,4) = 0.0;
        A(2*i+1,5) = 0.0;
        A(2*i+1,6) = -x2[i].x() * x1[i].x();
        A(2*i+1,7) = -x2[i].x() * x1[i].y();
        A(2*i+1,8) = -x2[i].x();

    }

    auto svd = Eigen::JacobiSVD<Matrix<16, 9>>(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix<Dynamic, Dynamic> Vt = svd.matrixV().transpose(); // TODO : Is this V transpose or V ??
    Matrix<Dynamic, Dynamic> Hflatten = Vt.row(8);

    // TODO : Check this. Transpose or not ?
    H = Eigen::Map<Matrix33>(Hflatten.data()).transpose();

    return true;
}

CML::scalar_t CML::Robust::RobustHomography::checkHypothesis(const Matrix33 &H21,
                                                           const Matrix33 &H12,
                                                           const List<DistortedVector2d> &x1,
                                                           const List<DistortedVector2d> &x2,
                                                           List<bool> &inliers) {

    // Code from ORB SLAM 2

    const int N = x1.size();

    const scalar_t h11 = H21(0,0);
    const scalar_t h12 = H21(0,1);
    const scalar_t h13 = H21(0,2);
    const scalar_t h21 = H21(1,0);
    const scalar_t h22 = H21(1,1);
    const scalar_t h23 = H21(1,2);
    const scalar_t h31 = H21(2,0);
    const scalar_t h32 = H21(2,1);
    const scalar_t h33 = H21(2,2);

    const scalar_t h11inv = H12(0,0);
    const scalar_t h12inv = H12(0,1);
    const scalar_t h13inv = H12(0,2);
    const scalar_t h21inv = H12(1,0);
    const scalar_t h22inv = H12(1,1);
    const scalar_t h23inv = H12(1,2);
    const scalar_t h31inv = H12(2,0);
    const scalar_t h32inv = H12(2,1);
    const scalar_t h33inv = H12(2,2);

    inliers.resize(N);

    scalar_t score = 0;

    const scalar_t th = 5.991;

    const scalar_t invSigmaSquare = 1.0 / (mSigma.f() * mSigma.f());

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const scalar_t u1 = x1[i].x();
        const scalar_t v1 = x1[i].y();
        const scalar_t u2 = x2[i].x();
        const scalar_t v2 = x2[i].y();

        // Reprojection error in first image
        // x2in1 = H12*x2

        const scalar_t w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const scalar_t u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const scalar_t v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const scalar_t squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const scalar_t chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th) {
            bIn = false;
        }
        else {
            score += th - chiSquare1;
        }
        // Reprojection error in second image
        // x1in2 = H21*x1

        const scalar_t w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const scalar_t u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const scalar_t v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const scalar_t squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const scalar_t chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th) {
            bIn = false;
        }
        else {
            score += th - chiSquare2;
        }

        inliers[i] = bIn;
    }

    return score;
}
