#ifndef CML_BOW_H
#define CML_BOW_H

#include <cml/config.h>
#include <cml/features/bow/BowVector.h>
#include <cml/features/bow/FeatureVector.h>
#include <cml/features/bow/TemplatedVocabulary.h>

namespace CML::Features {

    class BoW {

    public:
        CML::Features::BowVector bowVec;
        CML::Features::FeatureVector featVec;

    };

}

#endif