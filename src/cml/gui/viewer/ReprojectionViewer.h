#ifndef CML_REPROJECTIONVIEWER_H
#define CML_REPROJECTIONVIEWER_H

#include "cml/config.h"
#include "cml/base/AbstractFunction.h"
#include "cml/base/AbstractSlam.h"

namespace CML {

    class ReprojectionViewer : public AbstractFunction {

    public:
        explicit ReprojectionViewer(AbstractSlam *slam);

        std::string getName() final {
            return "Reprojection Viewer";
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final;

        void viewOnReconstruction(DrawBoard &drawBoard) final;


    private:
        AbstractSlam *mSlam;

    };

}

#endif