// Copyright (c) 2009, V. Lepetit, EPFL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
//   either expressed or implied, of the FreeBSD Project.

#ifndef epnp_h
#define epnp_h

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/cornerTracker/CornerMatcher.h>

namespace CML::Optimization {

    class EPnPInternal {

    protected:
        EPnPInternal();

        ~EPnPInternal();

        void set_internal_parameters(const scalar_t uc, const scalar_t vc,
                                     const scalar_t fu, const scalar_t fv);

        void set_maximum_number_of_correspondences(const int n);

        void reset_correspondences();

        void add_correspondence(const scalar_t X, const scalar_t Y, const scalar_t Z,
                                const scalar_t u, const scalar_t v);

        scalar_t compute_pose(Matrix33 &R, Vector3 &t);

        void relative_error(scalar_t &rot_err, scalar_t &transl_err,
                            const scalar_t Rtrue[3][3], const scalar_t ttrue[3],
                            const scalar_t Rest[3][3], const scalar_t test[3]);

        scalar_t reprojection_error(const scalar_t R[3][3], const scalar_t t[3]);

    private:
        void choose_control_points();

        void compute_barycentric_coordinates();

        void fill_M(Matrix<Dynamic, Dynamic> &M, const int row, const scalar_t *alphas, const scalar_t u, const scalar_t v);

        void compute_ccs(const scalar_t *betas, const scalar_t *ut);

        void compute_pcs();

        void solve_for_sign();

        void find_betas_approx_1(const Matrix<Dynamic, Dynamic> &L_6x10, const Matrix<Dynamic, Dynamic> &Rho, scalar_t *betas);

        void find_betas_approx_2(const Matrix<Dynamic, Dynamic> &L_6x10, const Matrix<Dynamic, Dynamic> &Rho, scalar_t *betas);

        void find_betas_approx_3(const Matrix<Dynamic, Dynamic> &L_6x10, const Matrix<Dynamic, Dynamic> &Rho, scalar_t *betas);

        void qr_solve(Matrix<Dynamic, Dynamic> &A, Matrix<Dynamic, Dynamic> &b, Matrix<Dynamic, Dynamic> &X);

        scalar_t dot(const scalar_t *v1, const scalar_t *v2);

        scalar_t dist2(const scalar_t *p1, const scalar_t *p2);

        void compute_rho(scalar_t *rho);

        void compute_L_6x10(const scalar_t *ut, scalar_t *l_6x10);

        void gauss_newton(const Matrix<Dynamic, Dynamic> &L_6x10, const Matrix<Dynamic, Dynamic> &Rho, scalar_t current_betas[4]);

        void compute_A_and_b_gauss_newton(const scalar_t *l_6x10, const scalar_t *rho,
                                          scalar_t cb[4], Matrix<6, 4> &A, Matrix<6, 1> &b);

        scalar_t compute_R_and_t(const scalar_t *ut, const scalar_t *betas,
                               scalar_t R[3][3], scalar_t t[3]);

        void estimate_R_and_t(scalar_t R[3][3], scalar_t t[3]);

        void copy_R_and_t(const scalar_t R_dst[3][3], const scalar_t t_dst[3],
                          Matrix33 &R_src, Vector3 &t_src);

        void mat_to_quat(const scalar_t R[3][3], scalar_t q[4]);


        scalar_t uc, vc, fu, fv;

        scalar_t *pws, *us, *alphas, *pcs;
        int maximum_number_of_correspondences;
        int number_of_correspondences;

        scalar_t cws[4][3], ccs[4][3];
    };

    class EPnP : public AbstractFunction, private EPnPInternal {

    public:
        EPnP(Ptr<AbstractFunction, NonNullable> parent) : AbstractFunction(parent) {};

        std::string getName() final {
            return "EPnP";
        }

        Optional<Camera> estimatePose(PFrame frame, List<Matching> matchings);

    };

    class EPnPRansac : private EPnPInternal {

    public:
        EPnPRansac(PFrame frame, List<Matching> matchings);

        void setRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4, float th2 = 5.991);

        Optional<Camera> iterate(int nIterations, bool &bNoMore, List<bool> &vbInliers, int &nInliers);

    protected:
        bool refine();

        void checkInliers();

    private:
        PFrame mFrame;
        List<Pair<int, Matching>> mMatchings;

        // Current Estimation
        Matrix33 mRi;
        Vector3 mti;
        List<bool> mvbInliersi;
        int mnInliersi = 0;

        // Current Ransac State
        int mnIterations = 0;
        List<bool> mvbBestInliers;
        int mnBestInliers = 0;

        Matrix33 mBestRi;
        Vector3 mBestti;

        // Refined
        Camera mRefinedTcw;
        List<bool> mvbRefinedInliers;
        int mnRefinedInliers;

        // Number of Correspondences
        int N = 0;

        // RANSAC probability
        double mRansacProb;

        // RANSAC min inliers
        int mRansacMinInliers;

        // RANSAC max iterations
        int mRansacMaxIts;

        // RANSAC expected inliers/total ratio
        float mRansacEpsilon;

        // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
        float mRansacTh;

        // RANSAC Minimun Set used at each iteration
        int mRansacMinSet;


    };

}

#endif