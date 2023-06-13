#ifndef CML_ORB_H
#define CML_ORB_H

#include <cml/config.h>
#include <cml/base/AbstractFunction.h>
#include <cml/features/Features.h>
#include <cml/features/bow/Bow.h>
#include <cml/capture/CaptureImage.h>
#include <cml/features/corner/FAST.h>
#if CML_HAVE_LIBZIP
#include <zip.h>
#endif

namespace CML::Features {

    typedef TemplatedVocabulary<Binary256Descriptor> ORBVocabulary;

#define CML_ORB_USEOPENCVFAST 0
#define CML_ORB_USEOPENCVIMAGE 0

    class ORB : public AbstractFunction {

    public:
        using Descriptor = Binary256Descriptor;

        explicit ORB(Ptr<AbstractFunction, Nullable> parent);

        inline std::string getName() final {
            return "ORB";
        }

        void loadVocabulary(const std::string &filename) {
            if (mVocabulary.isNotNull()) {
                return;
            }
            if (hasEnding(filename, ".txt")) {
                mVocabulary = new ORBVocabulary();
                mVocabulary->loadFromTextFile(filename);
                if (mVocabulary->empty()) {
                    throw std::runtime_error("Can't load vocabulary");
                }
            } else {
#if CML_HAVE_LIBZIP
                mVocabulary = new ORBVocabulary();
                mVocabulary->loadFromText(readWholeZipFile(filename, "ORBvoc.txt", true));
                if (mVocabulary->empty()) {
                    throw std::runtime_error("Can't load vocabulary");
                }
#else
                assertThrow(false, "Don't have libzip");
#endif
            }
        }

        const ORBVocabulary& getVocabulary() {
            return *mVocabulary.p();
        }

        void compute(const CaptureImage &frame);

        void compute(const FloatImage &image);

        void computeBow(Ptr<BoW, Nullable> &bow) {
            if (bow.isNull()) {
                bow = new BoW();
            }
            mVocabulary->transform(mDescriptors,bow->bowVec,bow->featVec,4);
        }

        List<Corner> &getCorners() {
            return mCorners;
        }

        List<Binary256Descriptor> &getDescriptors() {
            return mDescriptors;
        }

        ORB *setNumFeatures(int n) {
            if (mNumCorner.i() == n) {
                return this;
            }
            mNumCorner.set(n);
            reinitialize();
            return this;
        }

    protected:
        void reinitialize();

        class ExtractorNode : public DeterministicallyHashable
        {
        public:
            ExtractorNode() : bNoMore(false) {

            }

            void divideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

            List<Corner> vKeys;
            Vector2i UL, UR, BL, BR;
            LinkedList<ExtractorNode>::iterator lit;
            bool bNoMore;
        };

        void computeKeyPointsOctTree();

        void distributeOctTree(const List<Corner>& vToDistributeKeys, List<Corner>& vResultKeys, const int &minX, const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);

        void computeDescriptors(const GrayImage& image, List<Corner>& keypoints, List<Binary256Descriptor>& descriptors);

        void computeDescriptorsWithTransposedImage(const GrayImage& image, List<Corner>& keypoints, List<Binary256Descriptor>& descriptors);


        Binary256Descriptor computeDescriptor(const Corner& kpt, const GrayImage& img);

        Binary256Descriptor computeDescriptorWithTransposedImage(const Corner& kpt, const GrayImage& img);


        void computeOrientation(const GrayImage & image, List<Corner>& keypoints, const List<int>& umax);

    private:
        Parameter mNumCorner = createParameter("num", 1024);
        Parameter mUseCache = createParameter("useCache", false);
        Parameter mBlur = createParameter("blur", true);

        Parameter mIncreaseThreshold = createParameter("increaseThreshold", true);
        Parameter mIniThFAST = createParameter("iniThFAST", 20);
        Parameter mMinThFAST = createParameter("minThFAST", 7);
        Parameter mNLevels = createParameter("nLevels", 1);
        Parameter mScaleFactor = createParameter("scaleFactor", 1.2f);

        List<int> mnFeaturesPerLevel;

        List<int> umax;

        List<float> mvScaleFactor;
        List<float> mvInvScaleFactor;
        List<float> mvLevelSigma2;
        List<float> mvInvLevelSigma2;

        Array2D<float> mFilter;

        List<Binary256Descriptor> desc;


        Ptr<ORBVocabulary, Nullable> mVocabulary;

        List<GrayImage> mImages;
        List<GrayImage> mBluredImages;
        List<FloatImage> mBluredImagesTmpA, mBluredImagesTmpB, mBluredImagesTmpC, mBluredImagesTmpD, mBluredImagesTmpE;
        List<List<Corner>> mAllKeypoints;
        List<Corner> mCorners;
        List<Corner> vToDistributeKeys;
        List<Binary256Descriptor> mDescriptors;

        FAST *mFast;


    };

}

#endif