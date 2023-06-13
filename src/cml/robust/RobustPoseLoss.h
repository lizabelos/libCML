//
// Created by belosth on 31/12/2019.
//

#ifndef CML_ROBUSTLOSS_H
#define CML_ROBUSTLOSS_H

#include "cml/config.h"
#include "cml/map/Camera.h"
#include "cml/features/cornerTracker/CornerMatcher.h"
#include "cml/robust/backend/Ransac.h"

namespace CML {

    namespace Robust {

        class RobustPoseLoss : public LossFunction<Camera, Matching> {

        public:
            virtual std::string getLossName() const = 0;

        };

        class SampsonPoseLoss : public RobustPoseLoss {

        public:
            scalar_t processLoss(const Camera &model, const Matching &data) const final;

            std::string getLossName() const final {
                return "Sampson";
            }

        };

        class ReprojectionPoseLoss : public RobustPoseLoss {

        public:
            scalar_t processLoss(const Camera &model, const Matching &data) const final;

            std::string getLossName() const final {
                return "Reprojection";
            }

        };

    }

}

#endif //CML_ROBUSTLOSS_H
