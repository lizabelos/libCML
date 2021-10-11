/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include "cml/optimization/EPnP.h"

CML::Optimization::EPnPInternal::EPnPInternal()
{
    maximum_number_of_correspondences = 0;
    number_of_correspondences = 0;

    pws = 0;
    us = 0;
    alphas = 0;
    pcs = 0;
}

CML::Optimization::EPnPInternal::~EPnPInternal()
{
    delete [] pws;
    delete [] us;
    delete [] alphas;
    delete [] pcs;
}

void CML::Optimization::EPnPInternal::set_internal_parameters(scalar_t uc, scalar_t vc, scalar_t fu, scalar_t fv)
{
    this->uc = uc;
    this->vc = vc;
    this->fu = fu;
    this->fv = fv;
}

void CML::Optimization::EPnPInternal::set_maximum_number_of_correspondences(int n)
{
    if (maximum_number_of_correspondences < n) {
        if (pws != 0) delete [] pws;
        if (us != 0) delete [] us;
        if (alphas != 0) delete [] alphas;
        if (pcs != 0) delete [] pcs;

        maximum_number_of_correspondences = n;
        pws = new scalar_t[3 * maximum_number_of_correspondences];
        us = new scalar_t[2 * maximum_number_of_correspondences];
        alphas = new scalar_t[4 * maximum_number_of_correspondences];
        pcs = new scalar_t[3 * maximum_number_of_correspondences];
    }
}

void CML::Optimization::EPnPInternal::reset_correspondences()
{
    number_of_correspondences = 0;
}

void CML::Optimization::EPnPInternal::add_correspondence(scalar_t X, scalar_t Y, scalar_t Z, scalar_t u, scalar_t v)
{
    pws[3 * number_of_correspondences    ] = X;
    pws[3 * number_of_correspondences + 1] = Y;
    pws[3 * number_of_correspondences + 2] = Z;

    us[2 * number_of_correspondences    ] = u;
    us[2 * number_of_correspondences + 1] = v;

    number_of_correspondences++;
}

void CML::Optimization::EPnPInternal::choose_control_points()
{
    // Take C0 as the reference points centroid:
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            cws[0][j] += pws[3 * i + j];

    for(int j = 0; j < 3; j++)
        cws[0][j] /= number_of_correspondences;


    // Take C1, C2, and C3 from PCA on the reference points:
    Matrix<Dynamic, Dynamic> PW0(number_of_correspondences, 3);

    Matrix<3, 3> PW0tPW0 = Matrix<3, 3>();
    Matrix<3, 1> DC      = Matrix<3, 1>();
    Matrix<3, 3> UCt     = Matrix<3, 3>();

    for(int i = 0; i < number_of_correspondences; i++)
        for(int j = 0; j < 3; j++)
            PW0(i, j) = pws[3 * i + j] - cws[0][j];

    PW0tPW0 = PW0.transpose() * PW0;
    auto svd = PW0tPW0.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    DC = svd.singularValues();
    UCt = svd.matrixU();
    // cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

    for(int i = 1; i < 4; i++) {
        scalar_t k = sqrt(DC[i - 1] / number_of_correspondences);
        for(int j = 0; j < 3; j++)
            cws[i][j] = cws[0][j] + k * UCt(i - 1, j);
    }
}

void CML::Optimization::EPnPInternal::compute_barycentric_coordinates()
{
    Matrix<Dynamic, Dynamic> CC     = Matrix<3, 3>();
    Matrix<Dynamic, Dynamic> CC_inv = Matrix<3, 3>();

    for(int i = 0; i < 3; i++)
        for(int j = 1; j < 4; j++)
            CC(i, j - 1) = cws[j][i] - cws[0][i];

    // cvInvert(&CC, &CC_inv, CV_SVD);
    CC_inv = CC.inverse();
    // scalar_t * ci = cc_inv;
    for(int i = 0; i < number_of_correspondences; i++) {
        scalar_t * pi = pws + 3 * i;
        scalar_t * a = alphas + 4 * i;

        for(int j = 0; j < 3; j++)
            a[1 + j] =
                    CC_inv(0, j) * (pi[0] - cws[0][0]) +
                    CC_inv(1, j) * (pi[1] - cws[0][1]) +
                    CC_inv(2, j) * (pi[2] - cws[0][2]);
        a[0] = 1.0f - a[1] - a[2] - a[3];
    }
}

void CML::Optimization::EPnPInternal::fill_M(Matrix<Dynamic, Dynamic> & M, const int row, const scalar_t * as, const scalar_t u, const scalar_t v)
{
    scalar_t * M1 = M.data() + row * 12;
    scalar_t * M2 = M1 + 12;

    for(int i = 0; i < 4; i++) {
        M1[3 * i    ] = as[i] * fu;
        M1[3 * i + 1] = 0.0;
        M1[3 * i + 2] = as[i] * (uc - u);

        M2[3 * i    ] = 0.0;
        M2[3 * i + 1] = as[i] * fv;
        M2[3 * i + 2] = as[i] * (vc - v);
    }
}

void CML::Optimization::EPnPInternal::compute_ccs(const scalar_t * betas, const scalar_t * ut)
{
    for(int i = 0; i < 4; i++)
        ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

    for(int i = 0; i < 4; i++) {
        const scalar_t * v = ut + 12 * (11 - i);
        for(int j = 0; j < 4; j++)
            for(int k = 0; k < 3; k++)
                ccs[j][k] += betas[i] * v[3 * j + k];
    }
}

void CML::Optimization::EPnPInternal::compute_pcs()
{
    for(int i = 0; i < number_of_correspondences; i++) {
        scalar_t * a = alphas + 4 * i;
        scalar_t * pc = pcs + 3 * i;

        for(int j = 0; j < 3; j++)
            pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
    }
}

CML::scalar_t CML::Optimization::EPnPInternal::compute_pose(Matrix33 &R, Vector3 &t)
{
    choose_control_points();
    compute_barycentric_coordinates();

    Matrix<Dynamic, Dynamic> M = Matrix<Dynamic, Dynamic>(2 * number_of_correspondences, 12);

    for(int i = 0; i < number_of_correspondences; i++)
        fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

    Matrix<12, 12> MtM = Matrix<12, 12>();
    Matrix<12, 1> D   = Matrix<12, 1>();
    Matrix<12, 12> Ut  = Matrix<12, 12>();

    MtM = M.transpose() * M;
    auto svd = MtM.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    D = svd.singularValues();
    Ut = svd.matrixU();
    //cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
    //cvReleaseMat(&M);

    Matrix<6, 10> L_6x10 = Matrix<6, 10>();
    Matrix<6, 1> Rho    = Matrix<6, 1>();

    compute_L_6x10(Ut.data(), L_6x10.data());
    compute_rho(Rho.data());

    scalar_t Betas[4][4], rep_errors[4];
    scalar_t Rs[4][3][3], ts[4][3];

    find_betas_approx_1(L_6x10, Rho, Betas[1]);
    gauss_newton(L_6x10, Rho, Betas[1]);
    rep_errors[1] = compute_R_and_t(Ut.data(), Betas[1], Rs[1], ts[1]);

    find_betas_approx_2(L_6x10, Rho, Betas[2]);
    gauss_newton(L_6x10, Rho, Betas[2]);
    rep_errors[2] = compute_R_and_t(Ut.data(), Betas[2], Rs[2], ts[2]);

    find_betas_approx_3(L_6x10, Rho, Betas[3]);
    gauss_newton(L_6x10, Rho, Betas[3]);
    rep_errors[3] = compute_R_and_t(Ut.data(), Betas[3], Rs[3], ts[3]);

    int N = 1;
    if (rep_errors[2] < rep_errors[1]) N = 2;
    if (rep_errors[3] < rep_errors[N]) N = 3;

    copy_R_and_t(Rs[N], ts[N], R, t);

    return rep_errors[N];
}

void CML::Optimization::EPnPInternal::copy_R_and_t(const scalar_t R_src[3][3], const scalar_t t_src[3],
                        Matrix33 &R_dst, Vector3 &t_dst)
{
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++)
            R_dst(i,j) = R_src[i][j];
        t_dst[i] = t_src[i];
    }
}

CML::scalar_t CML::Optimization::EPnPInternal::dist2(const scalar_t * p1, const scalar_t * p2)
{
    return
            (p1[0] - p2[0]) * (p1[0] - p2[0]) +
            (p1[1] - p2[1]) * (p1[1] - p2[1]) +
            (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

CML::scalar_t CML::Optimization::EPnPInternal::dot(const scalar_t * v1, const scalar_t * v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

CML::scalar_t CML::Optimization::EPnPInternal::reprojection_error(const scalar_t R[3][3], const scalar_t t[3])
{
    scalar_t sum2 = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        scalar_t * pw = pws + 3 * i;
        scalar_t Xc = dot(R[0], pw) + t[0];
        scalar_t Yc = dot(R[1], pw) + t[1];
        scalar_t inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        scalar_t ue = uc + fu * Xc * inv_Zc;
        scalar_t ve = vc + fv * Yc * inv_Zc;
        scalar_t u = us[2 * i], v = us[2 * i + 1];

        sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
    }

    return sum2 / number_of_correspondences;
}

void CML::Optimization::EPnPInternal::estimate_R_and_t(scalar_t R[3][3], scalar_t t[3])
{
    scalar_t pc0[3], pw0[3];

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for(int i = 0; i < number_of_correspondences; i++) {
        const scalar_t * pc = pcs + 3 * i;
        const scalar_t * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            pc0[j] += pc[j];
            pw0[j] += pw[j];
        }
    }
    for(int j = 0; j < 3; j++) {
        pc0[j] /= number_of_correspondences;
        pw0[j] /= number_of_correspondences;
    }

    Matrix<3, 3> ABt   = Matrix<3, 3>::Zero();
    Matrix<3, 1> ABt_D = Matrix<3, 1>();
    Matrix<3, 3> ABt_U = Matrix<3, 3>();
    Matrix<3, 3> ABt_V = Matrix<3, 3>();

    for(int i = 0; i < number_of_correspondences; i++) {
        scalar_t * pc = pcs + 3 * i;
        scalar_t * pw = pws + 3 * i;

        for(int j = 0; j < 3; j++) {
            ABt(0, j) += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
            ABt(1, j) += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
            ABt(2, j) += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
        }
    }

    auto svd = ABt.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    ABt_D = svd.singularValues();
    ABt_U = svd.matrixU();
    ABt_V = svd.matrixV();

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R[i][j] = dot(ABt_U.data() + 3 * i, ABt_V.data() + 3 * j);

    const scalar_t det =
            R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
            R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

    if (det < 0) {
        R[2][0] = -R[2][0];
        R[2][1] = -R[2][1];
        R[2][2] = -R[2][2];
    }

    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);
}

void CML::Optimization::EPnPInternal::solve_for_sign()
{
    if (pcs[2] < 0.0) {
        for(int i = 0; i < 4; i++)
            for(int j = 0; j < 3; j++)
                ccs[i][j] = -ccs[i][j];

        for(int i = 0; i < number_of_correspondences; i++) {
            pcs[3 * i    ] = -pcs[3 * i];
            pcs[3 * i + 1] = -pcs[3 * i + 1];
            pcs[3 * i + 2] = -pcs[3 * i + 2];
        }
    }
}

CML::scalar_t CML::Optimization::EPnPInternal::compute_R_and_t(const scalar_t * ut, const scalar_t * betas,
                             scalar_t R[3][3], scalar_t t[3])
{
    compute_ccs(betas, ut);
    compute_pcs();

    solve_for_sign();

    estimate_R_and_t(R, t);

    return reprojection_error(R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void CML::Optimization::EPnPInternal::find_betas_approx_1(const Matrix<Dynamic, Dynamic> & L_6x10, const Matrix<Dynamic, Dynamic> & Rho,
                               scalar_t * betas)
{

    Matrix<6, 4> L_6x4 = Matrix<6, 4>();
    Matrix<4, 1> B4    = Matrix<4, 1>();

    for(int i = 0; i < 6; i++) {
        L_6x4(i, 0) = L_6x10(i, 0);
        L_6x4(i, 1) = L_6x10(i, 1);
        L_6x4(i, 2) = L_6x10(i, 3);
        L_6x4(i, 3) = L_6x10(i, 6);
    }

    auto svd = L_6x4.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    B4 = svd.solve(Rho);
    //cvSolve(&L_6x4, Rho, &B4, CV_SVD);

    if (B4[0] < 0) {
        betas[0] = sqrt(-B4[0]);
        betas[1] = -B4[1] / betas[0];
        betas[2] = -B4[2] / betas[0];
        betas[3] = -B4[3] / betas[0];
    } else {
        betas[0] = sqrt(B4[0]);
        betas[1] = B4[1] / betas[0];
        betas[2] = B4[2] / betas[0];
        betas[3] = B4[3] / betas[0];
    }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void CML::Optimization::EPnPInternal::find_betas_approx_2(const Matrix<Dynamic, Dynamic> & L_6x10, const Matrix<Dynamic, Dynamic> & Rho,
                               scalar_t * betas)
{
    Matrix<6, 3> L_6x3  = Matrix<6, 3>(6, 3);
    Matrix<3, 1> B3     = Matrix<3, 1>(3, 1);

    for(int i = 0; i < 6; i++) {
        L_6x3(i, 0) = L_6x10(i, 0);
        L_6x3(i, 1) = L_6x10(i, 1);
        L_6x3(i, 2) = L_6x10(i, 2);
    }

    auto svd = L_6x3.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    B3 = svd.solve(Rho);
    // cvSolve(&L_6x3, Rho, &B3, CV_SVD);

    if (B3[0] < 0) {
        betas[0] = sqrt(-B3[0]);
        betas[1] = (B3[2] < 0) ? sqrt(-B3[2]) : 0.0;
    } else {
        betas[0] = sqrt(B3[0]);
        betas[1] = (B3[2] > 0) ? sqrt(B3[2]) : 0.0;
    }

    if (B3[1] < 0) betas[0] = -betas[0];

    betas[2] = 0.0;
    betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void CML::Optimization::EPnPInternal::find_betas_approx_3(const Matrix<Dynamic, Dynamic> & L_6x10, const Matrix<Dynamic, Dynamic> & Rho,
                               scalar_t * betas)
{
    Matrix<6, 5> L_6x5 = Matrix<6, 5>(6, 5);
    Matrix<5, 1> B5    = Matrix<5, 1>(5, 1);

    for(int i = 0; i < 6; i++) {
        L_6x5(i, 0) = L_6x10(i, 0);
        L_6x5(i, 1) = L_6x10(i, 1);
        L_6x5(i, 2) = L_6x10(i, 2);
        L_6x5(i, 3) = L_6x10(i, 3);
        L_6x5(i, 4) = L_6x10(i, 4);
    }

    auto svd = L_6x5.jacobiSvd(Eigen::ComputeFullV | Eigen::ComputeFullU);
    B5 = svd.solve(Rho);

    if (B5[0] < 0) {
        betas[0] = sqrt(-B5[0]);
        betas[1] = (B5[2] < 0) ? sqrt(-B5[2]) : 0.0;
    } else {
        betas[0] = sqrt(B5[0]);
        betas[1] = (B5[2] > 0) ? sqrt(B5[2]) : 0.0;
    }
    if (B5[1] < 0) betas[0] = -betas[0];
    betas[2] = B5[3] / betas[0];
    betas[3] = 0.0;
}

void CML::Optimization::EPnPInternal::compute_L_6x10(const scalar_t * ut, scalar_t * l_6x10)
{
    const scalar_t * v[4];

    v[0] = ut + 12 * 11;
    v[1] = ut + 12 * 10;
    v[2] = ut + 12 *  9;
    v[3] = ut + 12 *  8;

    scalar_t dv[4][6][3];

    for(int i = 0; i < 4; i++) {
        int a = 0, b = 1;
        for(int j = 0; j < 6; j++) {
            dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
            dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
            dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

            b++;
            if (b > 3) {
                a++;
                b = a + 1;
            }
        }
    }

    for(int i = 0; i < 6; i++) {
        scalar_t * row = l_6x10 + 10 * i;

        row[0] =        dot(dv[0][i], dv[0][i]);
        row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
        row[2] =        dot(dv[1][i], dv[1][i]);
        row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
        row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
        row[5] =        dot(dv[2][i], dv[2][i]);
        row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
        row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
        row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
        row[9] =        dot(dv[3][i], dv[3][i]);
    }
}

void CML::Optimization::EPnPInternal::compute_rho(scalar_t * rho)
{
    rho[0] = dist2(cws[0], cws[1]);
    rho[1] = dist2(cws[0], cws[2]);
    rho[2] = dist2(cws[0], cws[3]);
    rho[3] = dist2(cws[1], cws[2]);
    rho[4] = dist2(cws[1], cws[3]);
    rho[5] = dist2(cws[2], cws[3]);
}

void CML::Optimization::EPnPInternal::compute_A_and_b_gauss_newton(const scalar_t * l_6x10, const scalar_t * rho,
                                        scalar_t betas[4], Matrix<6, 4> & A, Matrix<6, 1> & b)
{
    for(int i = 0; i < 6; i++) {
        const scalar_t * rowL = l_6x10 + i * 10;
        scalar_t * rowA = A.data() + i * 4;

        rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
        rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
        rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
        rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

        b(i, 0) = rho[i] -
                        (
                                rowL[0] * betas[0] * betas[0] +
                                rowL[1] * betas[0] * betas[1] +
                                rowL[2] * betas[1] * betas[1] +
                                rowL[3] * betas[0] * betas[2] +
                                rowL[4] * betas[1] * betas[2] +
                                rowL[5] * betas[2] * betas[2] +
                                rowL[6] * betas[0] * betas[3] +
                                rowL[7] * betas[1] * betas[3] +
                                rowL[8] * betas[2] * betas[3] +
                                rowL[9] * betas[3] * betas[3]
                        );
    }
}

void CML::Optimization::EPnPInternal::gauss_newton(const Matrix<Dynamic, Dynamic> & L_6x10, const Matrix<Dynamic, Dynamic> & Rho, scalar_t betas[4])
{
    const int iterations_number = 5;

    Matrix<6, 4> A = Matrix<6, 4>();
    Matrix<6, 1> B = Matrix<6, 1>();
    Matrix<4, 1> X = Matrix<4, 1>();

    for(int k = 0; k < iterations_number; k++) {
        compute_A_and_b_gauss_newton(L_6x10.data(), Rho.data(), betas, A, B);
        X = A.householderQr().solve(B);
        // qr_solve(&A, &B, &X);

        for(int i = 0; i < 4; i++)
            betas[i] += X[i];
    }
}

void CML::Optimization::EPnPInternal::qr_solve(Matrix<Dynamic, Dynamic> & A, Matrix<Dynamic, Dynamic> & b, Matrix<Dynamic, Dynamic> & X)
{
    static int max_nr = 0;
    static scalar_t * A1, * A2;

    const int nr = A.rows();
    const int nc = A.cols();

    if (max_nr != 0 && max_nr < nr) {
        delete [] A1;
        delete [] A2;
    }
    if (max_nr < nr) {
        max_nr = nr;
        A1 = new scalar_t[nr];
        A2 = new scalar_t[nr];
    }

    scalar_t * pA = A.data(), * ppAkk = pA;
    for(int k = 0; k < nc; k++) {
        scalar_t * ppAik = ppAkk, eta = fabs(*ppAik);
        for(int i = k + 1; i < nr; i++) {
            scalar_t elt = fabs(*ppAik);
            if (eta < elt) eta = elt;
            ppAik += nc;
        }

        if (eta == 0) {
            A1[k] = A2[k] = 0.0;
            logger.error("God damnit, A is singular, this shouldn't happen.");
            return;
        } else {
            scalar_t * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
            for(int i = k; i < nr; i++) {
                *ppAik *= inv_eta;
                sum += *ppAik * *ppAik;
                ppAik += nc;
            }
            scalar_t sigma = sqrt(sum);
            if (*ppAkk < 0)
                sigma = -sigma;
            *ppAkk += sigma;
            A1[k] = sigma * *ppAkk;
            A2[k] = -eta * sigma;
            for(int j = k + 1; j < nc; j++) {
                scalar_t * ppAik = ppAkk, sum = 0;
                for(int i = k; i < nr; i++) {
                    sum += *ppAik * ppAik[j - k];
                    ppAik += nc;
                }
                scalar_t tau = sum / A1[k];
                ppAik = ppAkk;
                for(int i = k; i < nr; i++) {
                    ppAik[j - k] -= tau * *ppAik;
                    ppAik += nc;
                }
            }
        }
        ppAkk += nc + 1;
    }

    // b <- Qt b
    scalar_t * ppAjj = pA, * pb = b.data();
    for(int j = 0; j < nc; j++) {
        scalar_t * ppAij = ppAjj, tau = 0;
        for(int i = j; i < nr; i++)	{
            tau += *ppAij * pb[i];
            ppAij += nc;
        }
        tau /= A1[j];
        ppAij = ppAjj;
        for(int i = j; i < nr; i++) {
            pb[i] -= tau * *ppAij;
            ppAij += nc;
        }
        ppAjj += nc + 1;
    }

    // X = R-1 b
    scalar_t * pX = X.data();
    pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
    for(int i = nc - 2; i >= 0; i--) {
        scalar_t * ppAij = pA + i * nc + (i + 1), sum = 0;

        for(int j = i + 1; j < nc; j++) {
            sum += *ppAij * pX[j];
            ppAij++;
        }
        pX[i] = (pb[i] - sum) / A2[i];
    }
}



void CML::Optimization::EPnPInternal::relative_error(scalar_t & rot_err, scalar_t & transl_err,
                          const scalar_t Rtrue[3][3], const scalar_t ttrue[3],
                          const scalar_t Rest[3][3],  const scalar_t test[3])
{
    scalar_t qtrue[4], qest[4];

    mat_to_quat(Rtrue, qtrue);
    mat_to_quat(Rest, qest);

    scalar_t rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
                           (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
                           (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
                           (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
                      sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    scalar_t rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
                           (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
                           (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
                           (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
                      sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

    rot_err = std::min(rot_err1, rot_err2);

    transl_err =
            sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
                 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
                 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
            sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void CML::Optimization::EPnPInternal::mat_to_quat(const scalar_t R[3][3], scalar_t q[4])
{
    scalar_t tr = R[0][0] + R[1][1] + R[2][2];
    scalar_t n4;

    if (tr > 0.0f) {
        q[0] = R[1][2] - R[2][1];
        q[1] = R[2][0] - R[0][2];
        q[2] = R[0][1] - R[1][0];
        q[3] = tr + 1.0f;
        n4 = q[3];
    } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
        q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
        q[1] = R[1][0] + R[0][1];
        q[2] = R[2][0] + R[0][2];
        q[3] = R[1][2] - R[2][1];
        n4 = q[0];
    } else if (R[1][1] > R[2][2]) {
        q[0] = R[1][0] + R[0][1];
        q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
        q[2] = R[2][1] + R[1][2];
        q[3] = R[2][0] - R[0][2];
        n4 = q[1];
    } else {
        q[0] = R[2][0] + R[0][2];
        q[1] = R[2][1] + R[1][2];
        q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
        q[3] = R[0][1] - R[1][0];
        n4 = q[2];
    }
    scalar_t scale = 0.5f / scalar_t(sqrt(n4));

    q[0] *= scale;
    q[1] *= scale;
    q[2] *= scale;
    q[3] *= scale;
}


CML::Optional<CML::Camera> CML::Optimization::EPnP::estimatePose(PFrame frame, List<Matching> matchings) {

    Matrix33 K = frame->getK(0);

    set_internal_parameters(K(0, 2), K(1, 2), K(0, 0), K(1, 1));

    int numPoint = 0;
    for (auto m : matchings) {
        if (m.getMapPoint().isNotNull()) {
            if (m.getMapPoint()->isGroup(getMap().MAPPED)) {
                numPoint++;
            }
        }
    }

    reset_correspondences();
    set_maximum_number_of_correspondences(numPoint);

    for (auto m : matchings) {
        if (m.getMapPoint().isNotNull()) {
            if (m.getMapPoint()->isGroup(getMap().MAPPED)) {
                auto p = m.getMapPoint()->getWorldCoordinate().absolute();
                auto ft = m.getFeaturePointA(frame);
                add_correspondence(p.x(), p.y(), p.z(), ft.x(), ft.y());
            }
        }
    }

    Matrix33 R;
    Vector3 t;

    getTimer().start();
    compute_pose(R, t);
    getTimer().stop();

    if (t.allFinite() && R.allFinite()) {
        return Camera(t, R);
    }

    return {};

}

CML::Optimization::EPnPRansac::EPnPRansac(PFrame frame, List<Matching> matchings) : mFrame(frame) {

   // mMatchings.resize(matchings.size());
    for (size_t i = 0; i < matchings.size(); i++) {
        mMatchings.emplace_back(Pair<int, Matching>(i, matchings[i]));
    }

    // Set camera calibration parameters
    Matrix33 K = frame->getK(0);
    set_internal_parameters(K(0, 2), K(1, 2), K(0, 0), K(1, 1));


    setRansacParameters();
}

void CML::Optimization::EPnPRansac::setRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2) {
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon;
    mRansacMinSet = minSet;

    N = mMatchings.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = std::max(1,std::min(nIterations,mRansacMaxIts));

    mRansacTh = th2;
}

CML::Optional<CML::Camera> CML::Optimization::EPnPRansac::iterate(int nIterations, bool &bNoMore, List<bool> &vbInliers, int &nInliers) {
    bNoMore = false;
    vbInliers.clear();
    nInliers=0;

    set_maximum_number_of_correspondences(mMatchings.size());

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return CML::Camera();
    }

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        reset_correspondences();

        // Get min set of points
        std::shuffle(mMatchings.begin(), mMatchings.end(), std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
        for(short i = 0; i < mRansacMinSet; ++i)
        {
            auto p = mMatchings[i].second.getMapPoint()->getWorldCoordinate().absolute();
            auto ft = mMatchings[i].second.getFeaturePointA(mFrame);
            add_correspondence(p.x(), p.y(), p.z(), ft.x(), ft.y());
        }

        // Compute camera pose
        compute_pose(mRi, mti);

        // Check inliers
        checkInliers();

        if(mnInliersi>=mRansacMinInliers)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                mBestRi = mRi;
                mBestti = mti;
            }

            if(refine())
            {
                nInliers = mnRefinedInliers;
                vbInliers = List<bool>();
                vbInliers.resize(mMatchings.size(), false);
                for(int i=0; i<N; i++)
                {
                    if(mvbRefinedInliers[i]) {
                        vbInliers[mMatchings[i].first] = true;
                    }
                }
                return mRefinedTcw;
            }

        }
    }

    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;
        if(mnBestInliers>=mRansacMinInliers)
        {
            nInliers=mnBestInliers;
            vbInliers = List<bool>(mMatchings.size(),false);
            for(int i=0; i<N; i++)
            {
                if(mvbBestInliers[i]) {
                    vbInliers[mMatchings[i].first] = true;
                }
            }
            return Camera(mBestti, mBestRi);
        }
    }

    return Optional<Camera>();
}


bool CML::Optimization::EPnPRansac::refine()
{

    reset_correspondences();
    for (auto [index, m] : mMatchings) {
        if (m.getMapPoint().isNotNull()) {
            //if (m.getMapPoint()->isGroup(getMap().MAPPED)) {
                auto p = m.getMapPoint()->getWorldCoordinate().absolute();
                auto ft = m.getFeaturePointA(mFrame);
                add_correspondence(p.x(), p.y(), p.z(), ft.x(), ft.y());
            //}
        }
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    checkInliers();

    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    if(mnInliersi>mRansacMinInliers)
    {
        mRefinedTcw = Camera(mti, mRi);
        return true;
    }

    return false;
}


void CML::Optimization::EPnPRansac::checkInliers()
{
    mnInliersi=0;

    for (int i = 0; i < N; i++) {

        auto m = mMatchings[i].second;
        UndistortedVector2d undistorted = m.getMapPoint()->getWorldCoordinate().project(mFrame->getCamera());
        scalar_t error2 = (mFrame->distort(undistorted, 0) - m.getFeaturePointA(mFrame).point0()).squaredNorm();

        scalar_t scaleFactor = m.getFeaturePointA(mFrame).processScaleFactorFromLevel();

        if(error2 < scaleFactor * scaleFactor * mRansacTh) // todo
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }

    }

}
