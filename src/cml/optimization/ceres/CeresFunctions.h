#ifndef CML_OPTIMIZATION_CERES_CERESFUNCTIONS
#define CML_OPTIMIZATION_CERES_CERESFUNCTIONS

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres/ceres.h>

namespace CML::Optimization::Ceres {

    class CeresReprojectionErrorXYZ {

    public:
        static ceres::AutoDiffCostFunction<CeresReprojectionErrorXYZ, 1, 3, 4, 3, 2, 4> *create() {
            return new ceres::AutoDiffCostFunction<CeresReprojectionErrorXYZ, 1, 3, 4, 3, 2, 4>(new CeresReprojectionErrorXYZ);
        }

        template <typename S> bool operator() (const S* const &_t, const S* const &_q, const S* const &_p, const S* const &_gt, const S* const &_intrinsics, S *_residual) const {
            Eigen::Matrix<S, 3, 1> t(_t[0], _t[1], _t[2]);
            Eigen::Quaternion<S> q(_q[0], _q[1], _q[2], _q[3]); // w, x, y, z
            Eigen::Matrix<S, 3, 1> p(_p[0], _p[1], _p[2]);
            Eigen::Matrix<S, 2, 1> gt(_gt[0], _gt[1]);
            S fx = _intrinsics[0];
            S fy = _intrinsics[1];
            S cx = _intrinsics[2];
            S cy = _intrinsics[3];

            Eigen::Matrix<S, 2, 1> undistortedProjection = (q * p + t).hnormalized();
            Eigen::Matrix<S, 2, 1> distortedProjection;
            distortedProjection(0) = undistortedProjection(0) * fx + cx;
            distortedProjection(1) = undistortedProjection(1) * fy + cy;
            Eigen::Matrix<S, 2, 1> residual = distortedProjection - gt;
            _residual[0] = residual.squaredNorm();
            //_residual[0] = ceres::abs(residual[0]);
            // _residual[1] = ceres::abs(residual[1]);

            return true;
        }

    private:
        CeresReprojectionErrorXYZ() {

        }

    };

    class FixedDistance {

    public:
        static ceres::AutoDiffCostFunction<FixedDistance, 1, 3, 3> *create(double gt) {
            return new ceres::AutoDiffCostFunction<FixedDistance, 1, 3, 3>(new FixedDistance(gt));
        }

        template <typename S> bool operator() (const S* const &raw_t1, const S* const &raw_t2, S *residual) const {

            Eigen::Matrix<S, 3, 1> t1(raw_t1[0], raw_t1[1], raw_t1[2]);
            Eigen::Matrix<S, 3, 1> t2(raw_t2[0], raw_t2[1], raw_t2[2]);
            S gt = S(mGT);

            residual[0] = ceres::abs((t1 - t2).squaredNorm() - (gt * gt));

            return true;
        }

    private:
        FixedDistance(double gt) : mGT(gt) {

        }

        double mGT;

    };

    template <int N> class FixedPoint {

    public:
        static ceres::AutoDiffCostFunction<FixedPoint, 1, N, N> *create() {
            return new ceres::AutoDiffCostFunction<FixedPoint, 1, N, N>(new FixedPoint());
        }

        template <typename S> bool operator() (const S* const &raw_t1, const S* const &raw_t2, S *residual) const {

            residual[0] = S(0);
            for (int i = 0; i < N; i++) {
                residual[0] = residual[0] + (raw_t1[i] - raw_t2[i]) * (raw_t1[i] - raw_t2[i]);
            }
            //if (abs(residual[0]) > S(0)) {
            //    residual[0] = sqrt(residual[0]);
            //}

            return true;
        }

    private:
        FixedPoint() {

        }

    };

    class AlignmentError {

    public:
        static ceres::AutoDiffCostFunction<AlignmentError, 1, 3, 3, 4, 1, 3> *create() {
            return new ceres::AutoDiffCostFunction<AlignmentError, 1, 3, 3, 4, 1, 3>(new AlignmentError());
        }

        template <typename S> static bool evaluate (const S* const &_t1, const S* const &_t2, const S* const &_q2, const S* const &_s2, S *_tfinal) {
            Eigen::Matrix<S, 3, 1> t1(_t1[0], _t1[1], _t1[2]);

            Eigen::Matrix<S, 3, 1> t2(_t2[0], _t2[1], _t2[2]);
            Eigen::Quaternion<S> q2(_q2[0], _q2[1], _q2[2], _q2[3]);

            q2.normalize();

            Eigen::Matrix<S, 3, 1> t = t1;
            t = t + t2;
            t = q2 * t;
            t = t * _s2[0];

            _tfinal[0] = t[0];
            _tfinal[1] = t[1];
            _tfinal[2] = t[2];

            return true;
        }

        template <typename S> bool operator() (const S* const &_t1, const S* const &_t2, const S* const &_q2, const S* const &_s2, const S* const &_tgt, S *_residual) const {

            Eigen::Matrix<S, 3, 1> t1(_t1[0], _t1[1], _t1[2]);

            Eigen::Matrix<S, 3, 1> t2(_t2[0], _t2[1], _t2[2]);
            Eigen::Quaternion<S> q2(_q2[0], _q2[1], _q2[2], _q2[3]);

            Eigen::Matrix<S, 3, 1> tgt(_tgt[0], _tgt[1], _tgt[2]);

            q2.normalize();

            Eigen::Matrix<S, 3, 1> t = t1;
            t = t + t2;
            t = q2 * t;
            t = t * _s2[0];

            _residual[0] = (t - tgt).squaredNorm();
            if (_residual[0] > S(1)) {
                _residual[0] = sqrt(_residual[0]);
            }

            return true;
        }

    private:
        AlignmentError() {

        }

    };

    class HardLimiterLossFunction : public ceres::LossFunction {

    public:
        explicit HardLimiterLossFunction(double limit, ceres::LossFunction *limitLoss = nullptr) : mLimit(limit), mLimitLoss(limitLoss) {

        }

        ~HardLimiterLossFunction() override {
            if (mLimitLoss != nullptr) {
                delete mLimitLoss;
            }
        }

        void Evaluate(double s, double rho[3]) const final {
            if (s > mLimit) {
                if (mLimitLoss == nullptr) {
                    rho[0] = 0;
                    rho[1] = 0;
                    rho[2] = 0;
                } else {
                    mLimitLoss->Evaluate(s, rho);
                }
            } else {
                rho[0] = s;
                rho[1] = 1.0;
                rho[2] = 0.0;
            }
        }

    private:
        double mLimit;
        ceres::LossFunction *mLimitLoss;

    };

    class ExpLoss : public ceres::LossFunction {

    public:
        void Evaluate(double s, double rho[3]) const final {
            rho[0] = exp(s);
            rho[1] = exp(s);
            rho[2] = exp(s);
        }

    };

}

#endif