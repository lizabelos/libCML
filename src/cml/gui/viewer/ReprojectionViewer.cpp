#include "cml/gui/viewer/ReprojectionViewer.h"
#include "cml/maths/Utils.h"

CML::ReprojectionViewer::ReprojectionViewer(AbstractSlam *slam) : AbstractFunction(slam), mSlam(slam) {

}

void CML::ReprojectionViewer::viewOnCapture(CML::DrawBoard &drawBoard, CML::PFrame frame) {
    if (mSlam->getMap().getFramesNumber() >= 2) {

        drawBoard.pointSize(3);

        List<scalar_t> allDepths;

        List<PPoint> mapPointsToDraw;


        for (auto mapPoint : frame->getGroupMapPoints(getMap().MAPPED)) {
            if (mapPoint->getIndirectApparitionNumber() >= 2) {
                mapPointsToDraw.emplace_back(mapPoint);
            }
        }

        for (const PPoint &mapPoint : mapPointsToDraw) {
            WorldPoint point = mapPoint->getWorldCoordinate();
            if (point.relative(frame->getCamera()).z() < 0) {
                continue;
            }
            allDepths.emplace_back(point.relative(frame->getCamera()).z());
        }



        scalar_t maxDepth = 10;

        if (allDepths.size() > 3) {
            maxDepth = median(allDepths) * 2;
        }


        for (const PPoint &mapPoint : mapPointsToDraw) {

            drawBoard.color(mapPoint->getColor().cast<float>());

            WorldPoint point = mapPoint->getWorldCoordinate();

            if (point.relative(frame->getCamera()).z() < 0) {
                continue;
            }

            Matrix<2, 1> projected = frame->distort(point.project(frame->getCamera()), 0);

            auto where = frame->getFeaturePoint(mapPoint);

            scalar_t colorDepth = point.relative(frame->getCamera()).z() / maxDepth;
            if (colorDepth > 1) colorDepth = 1;

            if (where.has_value()) {
                drawBoard.color(colorDepth, 1, 1.0 - colorDepth);
                if ((projected - where.value().point0()).norm() > 1) {
                    // Vector2 projectedForView = where.value().point0() + (projected - where.value().point0()) * 10;
                    Vector2 projectedForView = where.value().point0();
                    drawBoard.lineWidth(3);
                    drawBoard.segment((Eigen::Vector2f) projectedForView.cast<float>(),(Eigen::Vector2f) where.value().point0().cast<float>());
                } else {
                    drawBoard.pointSize(4);
                    drawBoard.point((Eigen::Vector2f) projected.cast<float>());
                }
            } else {
                drawBoard.pointSize(2);
                drawBoard.color(colorDepth, 0, 1.0 - colorDepth);
                drawBoard.point((Eigen::Vector2f) projected.cast<float>());
            }

        }
    }

}

void CML::ReprojectionViewer::viewOnReconstruction(CML::DrawBoard &drawBoard) {
    AbstractFunction::viewOnReconstruction(drawBoard);
}
