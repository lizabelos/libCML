#include "cml/robust/RobustPoseLoss.h"

CML::scalar_t CML::Robust::SampsonPoseLoss::processLoss(const Camera &model, const Matching &matching) const {
    auto E = (matching.getFrameB()->getCamera().to(model)).essential();

    UndistortedVector2d a = matching.noAssertGetUndistortedA(0);
    UndistortedVector2d b = matching.noAssertGetUndistortedB(0);

    Vector3 E_a = E * a.homogeneous();
    Vector3 Et_b = E.transpose() * b.homogeneous();

    scalar_t v = b.homogeneous().dot(E_a);

    scalar_t sampson = (v * v) / sqrt(E_a(0) * E_a(0) + E_a(1) * E_a(1) + Et_b(0) * Et_b(0) + Et_b(1) * Et_b(1));

    return sampson;
}

CML::scalar_t CML::Robust::ReprojectionPoseLoss::processLoss(const Camera &model, const Matching &matching) const {
    Vector2 projectionA = model.project(matching.getMapPoint()->getWorldCoordinate().absolute());
    return (projectionA - matching.noAssertGetUndistortedA(0)).norm();
}

