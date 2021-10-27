#ifndef CML_DSOFRAME_H
#define CML_DSOFRAME_H

#include <cml/config.h>
#include <cml/map/PrivateData.h>
#include <cml/optimization/dso/DSOResidual.h>
#include <cml/map/Exposure.h>

namespace CML {

    namespace Optimization {

        EIGEN_STRONG_INLINE Camera cameraOf(const Sophus::SE3<CML::scalar_t> &se3) {
            return Camera(se3.translation(), Quaternion(se3.unit_quaternion().w(), se3.unit_quaternion().x(), se3.unit_quaternion().y(), se3.unit_quaternion().z()));
        }

        class DSOFrame : public PrivateDataStructure {

            friend class DSOContext;

        public:
            DSOFrame() {
                step.setZero();
            }

            int id = -1;
            int keyid = -1;

            Vector<8> delta = Vector<8>::Zero();
            Vector<8> delta_prior = Vector<8>::Zero();
            Vector<8> prior = Vector<8>::Zero();

            Vector<10> prior_zero = Vector<10>::Zero();

            scalar_t frameEnergyTH = 8 * 8 * 8;

            Matrix<6, 6> nullspaces_pose = Matrix<6, 6>::Zero();
            Matrix<4, 2> nullspaces_affine = Matrix<4, 2>::Zero();
            Vector<6> nullspaces_scale = Vector<6>::Zero();

            SE3 PRE_worldToCam;
            SE3 PRE_camToWorld;

            scalar_t ab_exposure = 0;

            bool flaggedForMarginalization = false;

            EIGEN_STRONG_INLINE SE3 get_worldToCam_evalPT() const {
                return worldToCam_evalPT;
            }

            EIGEN_STRONG_INLINE Vector<10> get_state_zero() const {
                return state_zero;
            }

            EIGEN_STRONG_INLINE Vector<10> get_state() const {
                return state;
            }

            EIGEN_STRONG_INLINE Vector<10> get_state_backup() const {
                return state_backup;
            }

            EIGEN_STRONG_INLINE Vector<10> get_state_scaled() const {
                return state_scaled;
            }

            EIGEN_STRONG_INLINE Vector<10> get_state_minus_stateZero() const {
                return get_state() - get_state_zero();
            }

            inline Vector<6> w2c_leftEps() const {
                return get_state_scaled().head<6>();
            }

            inline void backupState() {
                state_backup = state;
            }

            inline void loadSateBackup(double scaleTrans, double scaleRot, double scaleA, double scaleB) {
                setState(state_backup, scaleTrans, scaleRot, scaleA, scaleB);
            }

            inline void doStepFromBackup(double scaleTrans, double scaleRot, double scaleA, double scaleB) {
                setState(state_backup + step, scaleTrans, scaleRot, scaleA, scaleB);
            }

            inline void setEvalPT(const SE3 &worldToCam_evalPT, const Vector<10> &state, double scaleTrans, double scaleRot, double scaleA, double scaleB)
            {

                this->worldToCam_evalPT = worldToCam_evalPT;
                setState(state, scaleTrans, scaleRot, scaleA, scaleB);
                setStateZero(state, scaleA, scaleB);
            }



            inline void setEvalPT_scaled(const SE3 &worldToCam_evalPT, const Exposure &aff_g2l, double scaleTrans, double scaleRot, double scaleA, double scaleB)
            {
                Vector<10> initial_state = Vector<10>::Zero();
                initial_state[6] = aff_g2l.getParameters()(0);
                initial_state[7] = aff_g2l.getParameters()(1);
                this->worldToCam_evalPT = worldToCam_evalPT;
                setStateScaled(initial_state, scaleTrans, scaleRot, scaleA, scaleB);
                setStateZero(this->get_state(), scaleA, scaleB);
            }



            void setState(const Vector<10> &state, double scaleTrans, double scaleRot, double scaleA, double scaleB) {
                this->state = state;
                state_scaled.segment<3>(0) = scaleTrans * state.segment<3>(0);
                state_scaled.segment<3>(3) = scaleRot * state.segment<3>(3);
                state_scaled[6] = scaleA * state[6];
                state_scaled[7] = scaleB * state[7];
                state_scaled[8] = scaleA * state[8];
                state_scaled[9] = scaleB * state[9];

                PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
                PRE_camToWorld = PRE_worldToCam.inverse();
                //setCurrentNullspace();
                //assertThrow(get_state_scaled()(7) < 1, "WAIT WHAT ?");

            }

            void setStateScaled(const Vector<10> &state_scaled, double scaleTrans, double scaleRot, double scaleA, double scaleB) {
                this->state_scaled = state_scaled;
                state.segment<3>(0) = state_scaled.segment<3>(0) / scaleTrans;
                state.segment<3>(3) = state_scaled.segment<3>(3) / scaleRot;
                state[6] = state_scaled[6] / scaleA;
                state[7] = state_scaled[7] / scaleB;
                state[8] = state_scaled[8] / scaleA;
                state[9] = state_scaled[9] / scaleB;

                PRE_worldToCam = SE3::exp(w2c_leftEps()) * get_worldToCam_evalPT();
                PRE_camToWorld = PRE_worldToCam.inverse();
                //setCurrentNullspace();

                //assertThrow(get_state_scaled()(7) < 1, "WAIT WHAT ?");

            }

            void setStateFromCamera(const Camera &camera, double scaleTrans, double scaleRot, double scaleA, double scaleB) {
                SE3 w2c = Sophus::SE3<CML::scalar_t>(camera.getQuaternion().toEigen(), camera.getTranslation());
                SE3 r2c = w2c * get_worldToCam_evalPT().inverse();
                SE3::Tangent tangent = r2c.log();

                Vector<10> state = get_state_scaled();
                state.segment<6>(0) = tangent;
                setStateScaled(state, scaleTrans, scaleRot, scaleA, scaleB);
            }

            void setStateZero(const Vector<10> &state_zero, double scaleA, double scaleB) {
                assert(state_zero.head<6>().squaredNorm() < 1e-20);

                this->state_zero = state_zero;

                //assertThrow(aff_g2l_0(scaleA, scaleB).getParameters()(1) < 1, "WAIT WHAT ?");

                for(int i=0;i<6;i++)
                {
                    Vector6 eps; eps.setZero(); eps[i] = 1e-3;
                    SE3 EepsP = SE3::exp(eps);
                    SE3 EepsM = SE3::exp(-eps);
                    SE3 w2c_leftEps_P_x0 = (get_worldToCam_evalPT() * EepsP) * get_worldToCam_evalPT().inverse();
                    SE3 w2c_leftEps_M_x0 = (get_worldToCam_evalPT() * EepsM) * get_worldToCam_evalPT().inverse();
                    nullspaces_pose.col(i) = (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log())/(2e-3);
                }
                //nullspaces_pose.topRows<3>() *= SCALE_XI_TRANS_INVERSE;
                //nullspaces_pose.bottomRows<3>() *= SCALE_XI_ROT_INVERSE;

                // scale change
                SE3 w2c_leftEps_P_x0 = (get_worldToCam_evalPT());
                w2c_leftEps_P_x0.translation() *= 1.00001;
                w2c_leftEps_P_x0 = w2c_leftEps_P_x0 * get_worldToCam_evalPT().inverse();
                SE3 w2c_leftEps_M_x0 = (get_worldToCam_evalPT());
                w2c_leftEps_M_x0.translation() /= 1.00001;
                w2c_leftEps_M_x0 = w2c_leftEps_M_x0 * get_worldToCam_evalPT().inverse();
                nullspaces_scale = (w2c_leftEps_P_x0.log() - w2c_leftEps_M_x0.log())/(2e-3);


                nullspaces_affine.setZero();
                nullspaces_affine.topLeftCorner<2,1>()  = Vector2(1,0);
                assert(ab_exposure > 0);
                nullspaces_affine.topRightCorner<2,1>() = Vector2(0, expf(aff_g2l_0(scaleA, scaleB).getParameters()(0)) * ab_exposure);
            }


            Exposure aff_g2l() const {
                return Exposure(ab_exposure, get_state_scaled()[6], get_state_scaled()[7]);
            }

            inline Exposure aff_g2l_0(double scaleA, double scaleB) const {
                return Exposure(ab_exposure, get_state_zero()[6] * scaleA, get_state_zero()[7] * scaleB);
            }

            inline float getB0(float scaleB) {
                return get_state_zero()[7] * scaleB;
            }

            const Set<Ptr<DSOResidual, NonNullable>> &getResiduals() {
                return residuals;
            }

            void setStep(Vector<10> &step) {
                for (int i = 0; i < 10; i++) {
                    if (!std::isfinite(step(i))) {
                        logger.error("Not finite step for frame");
                        this->step.setZero();
                        return;
                    }
                }
                this->step = step;
            }

            const Vector<10> &getStep() {
                return step;
            }

            const Set<PPoint, Hasher> &getPoints() const {
                return points;
            }

            int getNumResidualsOut() const {
                return numResidualsOut;
            }

            int getNumMarginalized() const {
                return numMarginalized;
            }

        protected:
            Set<Ptr<DSOResidual, NonNullable>> residuals;
            Set<PPoint, Hasher> points;
            int numResidualsOut = 0;
            int numMarginalized = 0;

        private:
            Vector<10> state = Vector<10>::Zero();
            Vector<10> state_zero = Vector<10>::Zero();
            Vector<10> state_backup = Vector<10>::Zero();
            Vector<10> state_scaled = Vector<10>::Zero();
            SE3 worldToCam_evalPT;
            Vector<10> step = Vector<10>::Zero();

        };

        class DSOFramePrecomputed {

        public:
            DSOFramePrecomputed() {

            }

            DSOFramePrecomputed(DSOFrame *hostData, DSOFrame *targetData) : mHostData(hostData), mTargetData(targetData) {
                precompute();
            }

            void precompute() {

                leftToLeft = mTargetData->PRE_worldToCam * mHostData->PRE_camToWorld;
                PRE_RTll = (leftToLeft.rotationMatrix());
                PRE_tTll = (leftToLeft.translation());

                leftToLeft_0 = mTargetData->get_worldToCam_evalPT() * mHostData->get_worldToCam_evalPT().inverse();
                PRE_RTll_0 = (leftToLeft_0.rotationMatrix());
                PRE_tTll_0 = (leftToLeft_0.translation());

                trialRefCamera = cameraOf(mHostData->PRE_worldToCam);
                trialTargetCamera = cameraOf(mTargetData->PRE_worldToCam);
                exposureTransition = mHostData->aff_g2l().to(mTargetData->aff_g2l());
                trialRefToTarget = trialRefCamera.to(trialTargetCamera);
            }

            DSOFrame *mHostData;
            DSOFrame *mTargetData;

            SE3 leftToLeft;
            Matrix33 PRE_RTll;
            Vector3 PRE_tTll;

            SE3 leftToLeft_0;
            Matrix33 PRE_RTll_0;
            Vector3 PRE_tTll_0;

            Camera trialRefCamera;
            Camera trialTargetCamera;
            ExposureTransition exposureTransition;
            Camera trialRefToTarget;

        };

    }

}

#endif