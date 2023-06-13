#ifndef CML_DSOTYPES_H
#define CML_DSOTYPES_H

#include <sophus/se3.hpp>

namespace CML {

    namespace Optimization {

        typedef Sophus::SE3<CML::scalar_t> SE3;

    }

}

#endif