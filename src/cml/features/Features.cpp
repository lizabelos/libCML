#include "cml/features/Features.h"

#include <vector>
#include <algorithm>
#include <execution>
#include <climits>
#include <memory>

std::vector<int> gDistances;

void initializaDescDistances(int N) {
    if (gDistances.size() < N * N) {
        gDistances.resize(N * N);
    }
    memset(&gDistances[0], 0, N * N * sizeof(int));
}

inline int &getDescDistance(int i, int j, int N) {
    return gDistances[i * N + j];
}

inline int *getDescDistances(int i, int N) {
    return &gDistances[i * N];
}

template <int DESCSIZE>
CML::BinaryDescriptor<DESCSIZE> CML::computeDistinctiveDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors) {

    // Compute distances between them
    const size_t N = descriptors.size();

    initializaDescDistances(N);

    for(size_t i=0;i<N;i++)
    {
        for(size_t j=i+1;j<N;j++)
        {
            int distij = descriptors[i].distance(descriptors[j]);
            getDescDistance(i,j,N)=distij;
            getDescDistance(j,i,N)=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int bestMedian = INT_MAX;
    int bestIdx = 0;
    for(size_t i = 0; i < N; i++)
    {
        auto dists = getDescDistances(i,N);
        std::nth_element(&dists[0], &dists[0] + N/2, &dists[0] + N);

        int m = dists[N/2];

        if(m < bestMedian)
        {
            bestMedian = m;
            bestIdx = i;
        }
    }

    return descriptors[bestIdx];
}


template <int DESCSIZE> CML::BinaryDescriptor<DESCSIZE> CML::computeMedianDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors)
{
    BinaryDescriptor<DESCSIZE> result;

    int sums[DESCSIZE * 8];

    for (size_t i = 0; i < descriptors.size(); i++) {

        for (size_t j = 0; j < DESCSIZE * 8; j++) {

            sums[j] += descriptors[i].getBit(j);

        }

    }

    int threshold = descriptors.size() < 2;

    for (size_t j = 0; j < DESCSIZE * 8; j++) {

        if (sums[j] < threshold) {
            result.clearBit(j);
        } else {
            result.setBit(j);
        }
    }

    return result;
}

template <int DESCSIZE> int CML::computeHashOfDescriptors(List<BinaryDescriptor<DESCSIZE>> &descriptors) {
    int h = 0;
    for (auto d : descriptors) {
        h += d.hash();
    }
    return h;
}

// specializations
template CML::BinaryDescriptor<32> CML::computeDistinctiveDescriptors(List<BinaryDescriptor<32>> &descriptors);
template CML::BinaryDescriptor<64> CML::computeDistinctiveDescriptors(List<BinaryDescriptor<64>> &descriptors);
template CML::BinaryDescriptor<128> CML::computeDistinctiveDescriptors(List<BinaryDescriptor<128>> &descriptors);
template CML::BinaryDescriptor<256> CML::computeDistinctiveDescriptors(List<BinaryDescriptor<256>> &descriptors);

template CML::BinaryDescriptor<32> CML::computeMedianDescriptors(List<BinaryDescriptor<32>> &descriptors);
template CML::BinaryDescriptor<64> CML::computeMedianDescriptors(List<BinaryDescriptor<64>> &descriptors);
template CML::BinaryDescriptor<128> CML::computeMedianDescriptors(List<BinaryDescriptor<128>> &descriptors);
template CML::BinaryDescriptor<256> CML::computeMedianDescriptors(List<BinaryDescriptor<256>> &descriptors);

template int CML::computeHashOfDescriptors(List<BinaryDescriptor<32>> &descriptors);
template int CML::computeHashOfDescriptors(List<BinaryDescriptor<64>> &descriptors);
template int CML::computeHashOfDescriptors(List<BinaryDescriptor<128>> &descriptors);
template int CML::computeHashOfDescriptors(List<BinaryDescriptor<256>> &descriptors);