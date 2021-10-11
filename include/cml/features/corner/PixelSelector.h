//
// Created by belosth on 03/01/2020.
//

#ifndef CML_PIXELSELECTOR_H
#define CML_PIXELSELECTOR_H

#include "cml/config.h"
#include "cml/features/Features.h"
#include "cml/base/AbstractFunction.h"

namespace CML::Features {

    /**
     * @article{engel2017direct,
     *   title={Direct sparse odometry},
     *   author={Engel, Jakob and Koltun, Vladlen and Cremers, Daniel},
     *   journal={IEEE transactions on pattern analysis and machine intelligence},
     *   volume={40},
     *   number={3},
     *   pages={611--625},
     *   year={2017},
     *   publisher={IEEE}
     * }
     */
    class PixelSelector : public AbstractFunction {

    public:
        PixelSelector(Ptr<AbstractFunction, Nullable> parent, int w, int h);
        ~PixelSelector() override;

        void compute(const CaptureImage &cp, List<Corner> &corners, List<float> &types, float density, bool allowFast, int recursionsLeft = 1, bool plot = false, float thFactor = 1);

        int makeMaps(const CaptureImage &cp, Array2D<float>& map_out, float density, int recursionsLeft=1, bool plot=false, float thFactor=1);;
        int makePixelStatus(const CaptureImage &cp, int lvl, Array2D<bool>& map, float desiredDensity, int &sparsityFactor, int recsLeft=5, float THFac = 1);


         inline void setPotential(int v) {
             currentPotential = v;
         }

    protected:
        int makePixelStatus(const CaptureImage &cp, int lvl, bool* map, float desiredDensity, int &sparsityFactor, int recsLeft=5, float THFac = 1);
        template<int pot> int gridMaxSelection(const CaptureImage &cp, int lvl, bool* map_out, float THFac);
        int gridMaxSelection(const CaptureImage &cp, int lvl, bool* map_out, int pot, float THFac);

        int makeMaps(const CaptureImage &cp, float* map_out, float density, int recursionsLeft=1, bool plot=false, float thFactor=1);
        void makeHists(const CaptureImage &cp);
        Eigen::Vector3i select(const CaptureImage &cp, float* map_out, int pot, float thFactor=1);

    private:
        bool allowFast;
        int currentPotential;
        unsigned char* randomPattern;
        int* gradHist;
        float* ths;
        float* thsSmoothed;
        int thsStep;

        const float minUseGrad_pixsel = 10;

    };

}

#endif //CML_PIXELSELECTOR_H
