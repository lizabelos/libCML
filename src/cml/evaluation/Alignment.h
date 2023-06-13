#ifndef CML_ALIGNMENT_H
#define CML_ALIGNMENT_H

#include <cml/config.h>

namespace CML::Evaluation {

    double align(const List<Camera> &input, const List<Optional<Camera>> &groundtruth, List<Camera> &output);

}

#endif