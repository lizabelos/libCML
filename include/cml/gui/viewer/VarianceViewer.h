#ifndef CML_VARIANCEVIEWER_H
#define CML_VARIANCEVIEWER_H

#include "cml/config.h"
#include "cml/base/AbstractFunction.h"


namespace CML {

    class VarianceViewer : public AbstractFunction {

    public:
        explicit VarianceViewer(Ptr<AbstractFunction, Nullable> parent);

        std::string getName() final {
            return "Variance Viewer";
        }

        void viewOnCapture(DrawBoard &drawBoard, PFrame frame) final;

        void viewOnReconstruction(DrawBoard &drawBoard) final;


    private:
        Parameter mFilterParameter = createParameter("Filter", 1.0f);

    };

}

#endif