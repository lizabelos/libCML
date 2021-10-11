#ifndef CML_BRIEF_H
#define CML_BRIEF_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/Features.h>
#include <cml/features/bow/Bow.h>
#include <cml/capture/CaptureImage.h>

namespace CML::Features {

    typedef TemplatedVocabulary<Binary256Descriptor> BRIEFVocabulary;

    class BRIEF : public AbstractFunction {

    public:
        using Descriptor = Binary256Descriptor;

        BRIEF(Ptr<AbstractFunction, NonNullable> parent, int threshold = 10) : AbstractFunction(parent) {
            mThreshold.set(threshold);
        }

        inline std::string getName() final {
            return "BRIEF";
        }

        void loadVocabulary(const std::string &filename) {
            mVocabulary = new BRIEFVocabulary();
            mVocabulary->loadFromTextFile(filename);
            if (mVocabulary->empty()) {
                throw std::runtime_error("Can't load vocabulary");
            }
        }


        void compute(const CaptureImage &frame, List<Corner> &corners, List <Binary256Descriptor> &descriptors);

        void compute(const CaptureImage &frame, List<Corner> &corners, List<Binary256Descriptor> &descriptors, Ptr<BoW, Nullable> &bow);

        int compute(PFrame frame, List<Binary256Descriptor> &descriptors);

    protected:
        Binary256Descriptor computeDescriptor(const Array2D<unsigned char> &frame, Corner &corner);

    private:
        Parameter mThreshold = createParameter("Threshold", 10);
        Ptr<BRIEFVocabulary, Nullable> mVocabulary;

    };

}

#endif