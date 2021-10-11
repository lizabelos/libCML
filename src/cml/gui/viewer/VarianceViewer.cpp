#include "cml/gui/viewer/VarianceViewer.h"

// http://www.andrewnoske.com/wiki/Code_-_heatmaps_and_color_gradients
bool getHeatMapColor(float value, float *red, float *green, float *blue)
{
    const int NUM_COLORS = 4;
    static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
    // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.

    int idx1;        // |-- Our desired color will be between these two indexes in "color".
    int idx2;        // |
    float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.

    if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
    else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
    else
    {
        value = value * (NUM_COLORS-1);        // Will multiply value by 3.
        idx1  = floor(value);                  // Our desired color will be after this index.
        idx2  = idx1+1;                        // ... and before this index (inclusive).
        fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
    }

    *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
    *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
    *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];

    return true;
}

CML::VarianceViewer::VarianceViewer(Ptr<AbstractFunction, Nullable> parent) : AbstractFunction(parent) {

}

void CML::VarianceViewer::viewOnCapture(CML::DrawBoard &drawBoard, CML::PFrame frame) {
    AbstractFunction::viewOnCapture(drawBoard, frame);
}

void CML::VarianceViewer::viewOnReconstruction(CML::DrawBoard &drawBoard) {
    // Vector<scalar_t> variances;
/*
    scalar_t varianceMax = 0, varianceMin = std::numeric_limits<scalar_t>::max();
    for (auto mapPoint : mMap.getMapPoints()) {
        if (mapPoint->isGroup(MAPPED)) {
            scalar_t variance = mapPoint->getVariance();
            if (variance > varianceMax) varianceMax = variance;
            if (variance < varianceMin) varianceMin = variance;
        }
        // variances.emplace_back(mapPoint->getVariance());
    }

    if (varianceMax == varianceMin) {
        return;
    }
*/
    // std::sort(variances.begin(), variances.end());

    for (auto mapPoint : getMap().getMapPoints()) {
        if (mapPoint->isGroup(getMap().MAPPED)) {

            scalar_t ratio = mapPoint->getUncertainty();
            if (ratio > 1) ratio = 1;
            // scalar_t ratio = (mapPoint->getVariance() - varianceMin) / (varianceMax - varianceMin);
            float r, g, b;
            getHeatMapColor(ratio, &r, &g, &b);

            if (ratio <= mFilterParameter.f()) {
                drawBoard.color(r, g, b);
                drawBoard.point((Eigen::Vector3f) mapPoint->getWorldCoordinate().absolute().cast<float>());
            }

        }
    }
}
