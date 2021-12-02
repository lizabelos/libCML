#ifndef CML_DSOPOINT_H
#define CML_DSOPOINT_H

#include <cml/config.h>
#include <cml/optimization/dso/DSOTypes.h>
#include <cml/map/PrivateData.h>
#include <cml/optimization/dso/DSOResidual.h>

namespace CML {

    namespace Optimization {

        typedef enum {
            IPS_GOOD=0,					// traced well and good
            IPS_OOB,					// OOB: end tracking & marginalize!
            IPS_OUTLIER,				// energy too high: if happens again: outlier!
            IPS_SKIPPED,				// traced well and good (but not actually traced).
            IPS_BADCONDITION,			// not traced because of bad condition.
            IPS_UNINITIALIZED           // not even traced once.
        } DSOTracerStatus;

        typedef enum {
            DSOPT_ACTIVE=0, DSOPT_INACTIVE, DSOPT_OUTLIER, DSOPT_OOB, DSOPT_MARGINALIZED
        } DSOPointStatus;

        class DSOPoint : public PrivateDataStructure {

            friend class DSOContext;

        public:

            DSOPoint() {
                // todo compute weights and other;
            }

            scalar_t colors[8]; // todo : pattern size

            /// BUNDLE ADJUSTMENT
            bool haveAF = false;
            float Hdd_accAF = 0;
            float bd_accAF = 0;
            Vector4f Hcd_accAF = Vector4f::Constant(0);
            bool haveLF = false;
            float Hdd_accLF = 0;
            float bd_accLF = 0;
            Vector4f Hcd_accLF = Vector4f::Constant(0);
            List<scalar_t> weights;
            Matrix22 gradH = Matrix22::Zero();

            float HdiF = 0;
            float bdSumF = 0;

            float deltaF = 0;
            float priorF = 0;

            scalar_t step = 0;

            float idepth_zero = 0;
            float idepth_backup = 0;

            bool hasDepthPrior = false;

            bool flaggedForMarginalization = false;

            DSOPointStatus status = DSOPT_ACTIVE;

            int numGoodResiduals = 0;

            const Set<DSOResidual*> &getResiduals() {
                return residuals;
            }

            void setInverseDepthHessian(float h) {
                idepth_hessian = h;
            }

            float getInverseDepthHessian() {
                return idepth_hessian;
            }

            void setMaxRelBaseline(float b) {
                maxRelBaseline = b;
            }

            float getMaxRelBaseline() {
                return maxRelBaseline;
            }

            void updatePointUncertainty(PPoint point, float scaledVarTH, float absVarTH, float minRelBS) {

                scalar_t idepth = point->getReferenceInverseDepth();

                scalar_t depth = 1.0 / idepth;
                scalar_t depth4 = depth * depth;
                depth4 *= depth4;
                scalar_t var = (1.0 / (idepth_hessian+0.01));
                point->setUncertainty(var);

                return;
            }

            Pair<DSOResidual*, DSOResidualState> &getLastResidual(int i) {
                return lastResiduals[i];
            }

            void resetLastResidual() {
                lastResiduals[0].first = 0;
                lastResiduals[0].second = DSORES_OOB;
                lastResiduals[1].first = 0;
                lastResiduals[1].second = DSORES_OOB;
            }

            void setLastResidual(DSOResidual *res, DSOResidualState state) {
                lastResiduals[1] = lastResiduals[0];
                setLastResidual(0, res, state);
            }

            void setLastResidual(int i, DSOResidual *res, DSOResidualState state) {
                if (i == 0 && state == DSORES_IN) {
                    DistortedVector2d distortedVector2D(res->getCenterProjectedTo().head<2>().cast<scalar_t>());
                    if (!res->elements.frame->isInside(distortedVector2D, 0, 0)) {
                        logger.fatal("Center projected to is not inside the frame, and state is DSORES_IN !");
                        abort();
                    }
                }
                lastResiduals[i].first = res;
                lastResiduals[i].second = state;
            }

            void setResidualState(DSOResidual* r, DSOResidualState state) {
                if(lastResiduals[0].first == r) {
                    lastResiduals[0].second = state;
                }
                else if(lastResiduals[1].first == r) {
                    lastResiduals[1].second = state;
                }
            }


        private:
            Set<DSOResidual*> residuals;
            Pair<DSOResidual*, DSOResidualState> lastResiduals[2];

            float idepth_hessian = 0;
            float maxRelBaseline = 0;



        };

    }

}

#endif