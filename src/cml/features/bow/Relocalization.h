#ifndef CML_RELOCALIZATION_H
#define CML_RELOCALIZATION_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/map/Map.h>

namespace CML::Features {

    class Relocalization : public AbstractFunction, public Map::Observer {

    public:
        explicit Relocalization(Ptr<AbstractFunction, NonNullable> parent, const AbstractVocabulary &vocabulary) : AbstractFunction(parent), mVocabulary(vocabulary) {
            getMap().subscribeObserver(this);
            mInvertedFile.resize(vocabulary.size());
            LOOPCLOSUREFRAMEGROUP = getMap().createFrameGroup("Loop Closure");
        }

        ~Relocalization() {
            getMap().removeObserver(this);
        }

        List<PFrame> detectRelocalizationCandidates(PFrame frame);

        void onFrameChangeGroup(Map &map, PFrame frame, int groupId, bool state) final;


    private:
        const AbstractVocabulary &mVocabulary;
        List<List<PFrame>> mInvertedFile;

    public:
        int LOOPCLOSUREFRAMEGROUP;

    };


}


#endif