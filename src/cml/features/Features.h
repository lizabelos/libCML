#ifndef CML_FEATURES_HEADER
#define CML_FEATURES_HEADER

#include <cmath>
#include <vector>
#include <bitset>

#include "cml/config.h"
#include "cml/maths/Utils.h"


namespace CML {

    class Descriptor {

    public:
        virtual ~Descriptor() {

        }

    };

    template <int N> class BinaryDescriptor : public Descriptor { // N is byte not bit

    public:
        static const int L = N;
        static const int NUMELEMENTS = L * 8;

        EIGEN_STRONG_INLINE BinaryDescriptor() {
            memset(mBinary, 0, N);
        }

        EIGEN_STRONG_INLINE BinaryDescriptor(void *binary) {
            memcpy(mBinary, binary, N);
        }

        EIGEN_STRONG_INLINE BinaryDescriptor(const std::string &s) {
            memset(mBinary, 0, N);

            unsigned char *values = (unsigned char*)mBinary;

            std::stringstream ss(s);
            std::istream_iterator<std::string> begin(ss);
            std::istream_iterator<std::string> end;
            std::vector<std::string> vstrings(begin, end);

            for (int i = 0; i < N; i++) {
                values[i] = std::stoi(vstrings[i]);
            }

        }

        EIGEN_STRONG_INLINE bool getBit(unsigned int index) const {
            unsigned int arrayIndex = index / 8;
            unsigned int bitIndex = index % 8;
            return (mBinary[arrayIndex] >> bitIndex) & 1;
        }

        EIGEN_STRONG_INLINE void setBit(unsigned int index) {
            unsigned int arrayIndex = index / 8;
            unsigned int bitIndex = index % 8;
            mBinary[arrayIndex] |= (uint8_t)1 << bitIndex;
        }

        EIGEN_STRONG_INLINE void clearBit(unsigned int index) {
            unsigned int arrayIndex = index / 8;
            unsigned int bitIndex = index % 8;
            mBinary[arrayIndex] &= ~((uint8_t)1 << bitIndex);
        }

        EIGEN_STRONG_INLINE int distance(const BinaryDescriptor<N> &other) const {

            const uint8_t  *binary8_a  = mBinary;
            const uint16_t *binary16_a = (uint16_t*)mBinary;
            const uint32_t *binary32_a = (uint32_t*)mBinary;
            const uint64_t *binary64_a = (uint64_t*)mBinary;

            const uint8_t  *binary8_b  = other.mBinary;
            const uint16_t *binary16_b = (uint16_t*)other.mBinary;
            const uint32_t *binary32_b = (uint32_t*)other.mBinary;
            const uint64_t *binary64_b = (uint64_t*)other.mBinary;

            int result = 0;
            int i = 0;

            while (i + 8 <= N) {
                result += __builtin_popcountll(binary64_a[i / 8] ^ binary64_b[i / 8]);
                i += 8;
            }

            while (i + 4 <= N) {
                result += __builtin_popcountll(binary32_a[i / 4] ^ binary32_b[i / 4]);
                i += 4;
            }

            while (i + 2 <= N) {
                result += __builtin_popcountll(binary16_a[i / 2] ^ binary16_b[i / 2]);
                i += 2;
            }

            while (i < N) {
                result += __builtin_popcountll(binary8_a[i] ^ binary8_b[i]);
                i++;
            }

            return result;
        }

        EIGEN_STRONG_INLINE std::string toString() const {
            std::string result;
            unsigned char *values = (unsigned char*)mBinary;

            for (int i = 0; i < N; i++) {
                if (i != 0) {
                    result += " ";
                }
                result += std::to_string((int)values[i]);
            }

            return result;
        }

        EIGEN_STRONG_INLINE const uint8_t *data() const {
            return mBinary;
        }

        EIGEN_STRONG_INLINE uint8_t *data() {
            return mBinary;
        }

        EIGEN_STRONG_INLINE static int distance(const BinaryDescriptor<N> &a, const BinaryDescriptor<N> &b) {
            return a.distance(b);
        }

        EIGEN_STRONG_INLINE static BinaryDescriptor<N> meanValue(const List<BinaryDescriptor<N>> &descriptors) {
            BinaryDescriptor<N> mean;
            if(descriptors.empty()) return mean;
            const int N2 = descriptors.size() / 2;

            List<int> counters;
            counters.resize(N * 8, 0);

            for(auto descriptor : descriptors)
            {
                for(int i = 0; i < N * 8; i++)
                {
                    if (descriptor.getBit(i)) counters[i]++;
                }
            }

            for(int i = 0; i < N * 8; i++)
            {
                if(counters[i] > N2) mean.setBit(i);
            }

            return mean;
        }

        EIGEN_STRONG_INLINE static BinaryDescriptor<N> meanValue(const List<const BinaryDescriptor<N>*> &descriptors) {
            BinaryDescriptor<N> mean;
            if(descriptors.empty()) return mean;
            const int N2 = descriptors.size() / 2;

            List<int> counters;
            counters.resize(N * 8, 0);

            for(auto descriptor : descriptors)
            {
                for(int i = 0; i < N * 8; i++)
                {
                    if (descriptor->getBit(i)) counters[i]++;
                }
            }

            for(int i = 0; i < N * 8; i++)
            {
                if(counters[i] > N2) mean.setBit(i);
            }

            return mean;
        }

        EIGEN_STRONG_INLINE int hash() {
            int h = 0;
            for (int i = 0; i < N; i++) {
                h += mBinary[i];
            }
            return h;
        }

        EIGEN_STRONG_INLINE size_t size() {
            return sizeof(uint8_t) * N;
        }

    private:
        uint8_t mBinary[N];

    };

    // todo : https://software.intel.com/sites/landingpage/IntrinsicsGuide/#text=popcnt&expand=4379,4381
    // avx popcnt

    using Binary8Descriptor = BinaryDescriptor<1>;
    using Binary16Descriptor = BinaryDescriptor<2>;
    using Binary32Descriptor = BinaryDescriptor<4>;
    using Binary64Descriptor = BinaryDescriptor<8>;
    using Binary128Descriptor = BinaryDescriptor<16>;
    using Binary256Descriptor = BinaryDescriptor<32>;
    using Binary512Descriptor = BinaryDescriptor<64>;
    using Binary1024Descriptor = BinaryDescriptor<128>;

    template <int DESCSIZE> BinaryDescriptor<DESCSIZE> computeDistinctiveDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors);

    template <int DESCSIZE> BinaryDescriptor<DESCSIZE> computeMedianDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors);

    template <int DESCSIZE> int computeHashOfDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors);

}

#endif // CML_FEATURES_HEADER