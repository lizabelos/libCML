#ifndef CML_DSORESIDUAL_H
#define CML_DSORESIDUAL_H

#include <cml/config.h>
#include <cml/map/MapObject.h>
#include <cml/map/Frame.h>

namespace CML {

    const int DSOMAXRESPERPOINT = 8;

    namespace Optimization {

        typedef enum {
            DSORES_IN = 0, DSORES_OOB, DSORES_OUTLIER
        } DSOResidualState;

        typedef enum {
            DSORES_ACTIVE, DSORES_LINEARIZED, DSORES_MARGINALIZED
        } DSOResidualMode;

        class DSORawResidualJacobian
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        public:
            DSORawResidualJacobian() {
                resF.setZero();
                Jpdxi[0].setZero();
                Jpdxi[1].setZero();
                Jpdc[0].setZero();
                Jpdc[1].setZero();
                Jpdd.setZero();
                JIdx[0].setZero();
                JIdx[1].setZero();
                JabF[0].setZero();
                JabF[1].setZero();
                JIdx2.setZero();
                JabJIdx.setZero();
                Jab2.setZero();
            }

            // ================== new structure: save independently =============.
            Vector8f resF; // todo : this is the pattern size here (or the pattern max size)

            // the two rows of d[x,y]/d[xi].
            Vector6f Jpdxi[2];			// 2x6

            // the two rows of d[x,y]/d[C].
            Vector4f Jpdc[2];			// 2x4

            // the two rows of d[x,y]/d[idepth].
            Vector2f Jpdd;				// 2x1

            // the two columns of d[r]/d[x,y].
            Vector8f JIdx[2];			// 9x2

            // = the two columns of d[r] / d[ab]
            Vector8f JabF[2];			// 9x2


            // = JIdx^T * JIdx (inner product). Only as a shorthand.
            Matrix22f JIdx2;				// 2x2
            // = Jab^T * JIdx (inner product). Only as a shorthand.
            Matrix22f JabJIdx;			// 2x2
            // = Jab^T * Jab (inner product). Only as a shorthand.
            Matrix22f Jab2;			// 2x2

        };


        class DSOResidual : public DeterministicallyHashable {

        public:
            DSOResidual(const OptimizationPair &p) : elements(p) {
                resetOOB();
                isLinearized = false;
                isActiveAndIsGoodNEW = false;
            };

            void resetOOB()
            {
                state_NewEnergy = state_energy = 0;
                state_NewState = DSOResidualState::DSORES_OUTLIER;
                state_state = DSOResidualState::DSORES_IN;
            };

            OptimizationPair elements;
            DSORawResidualJacobian efsJ, rJ;

            bool isLinearized = false;

            double state_energy = 0;
            double state_NewEnergy = 0;
            double state_NewEnergyWithOutlier = 0;

            Vector8f JpJdF = Vector8f::Zero();

            bool isActiveAndIsGoodNEW = false;

            Vectorf<DSOMAXRESPERPOINT> res_toZeroF;

            void setCenterProjectedTo(const Vector3 &centerProjectedTo) {
                mCenterProjectedTo = centerProjectedTo;
            }

            Vector3 getCenterProjectedTo() {
                return mCenterProjectedTo;
            }

            DSOResidualState getState()  {
                return state_state;
            }

            DSOResidualState getNewState() {
                return state_NewState;
            }

            void applyNewState() {
                state_state = state_NewState;
            }

            void setNewState(DSOResidualState state) {
                if (state == DSORES_IN) abortIfWrongCenterProjectedTo();
                state_NewState = state;
            }

            void setState(DSOResidualState state) {
                if (state == DSORES_IN) abortIfWrongCenterProjectedTo();
                state_state = state;
            }


        protected:
            void abortIfWrongCenterProjectedTo() {
                DistortedVector2d distortedVector2D(mCenterProjectedTo.head<2>().cast<scalar_t>());
                if (!elements.frame->isInside(distortedVector2D, 0, 0)) {
                    logger.fatal("DSORES_IN but center projected to not in");
                    abort();
                }
            }


        private:
            Vector3 mCenterProjectedTo;
            DSOResidualState state_state = DSORES_OUTLIER;
            DSOResidualState state_NewState = DSORES_OUTLIER;


        };

    }

}


#endif