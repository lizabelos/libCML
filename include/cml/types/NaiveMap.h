#ifndef CML_NAIVEMAP_H
#define CML_NAIVEMAP_H

#include <vector>
#include <array>
#include <stdint.h>
#include <random>
#include <stdexcept>
#include <assert.h>

namespace CML {
    template<typename K, typename V>
    class NaiveMapNode {
    public:
        NaiveMapNode() : hash(-1) {
        }

        NaiveMapNode(K _first, V _second, int64_t _hash) : first(_first), second(_second), hash(_hash) {
        }

        K first;
        V second;
        int64_t hash;

        template <size_t I>
        auto& get() & {
            if constexpr (I == 0) return first;
            else if constexpr (I == 1) return second;
        }

        template <size_t I>
        auto const& get() const& {
            if constexpr (I == 0) return first;
            else if constexpr (I == 1) return second;
        }

        template <size_t I>
        auto&& get() && {
            if constexpr (I == 0) return std::move(first);
            else if constexpr (I == 1) return std::move(second);
        }


    };

    template<typename K, typename V, typename H, int bucketSize = 256>
    class NaiveMap {

    public:
        NaiveMap() {
            for (int i = 0; i < bucketSize; i++) {
                mBuckets[i].reserve(1);
            }
        }

        void reserve(int size) {
            size = size / (bucketSize / 2);
            for (int i = 0; i < bucketSize; i++) {
                mBuckets[i].reserve(size);
            }
        }

        void insert(const K &key, const V &value) {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            // check if key already exists
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    mBuckets[bucket][i].second = value;
                    return;
                }
            }
            // insert new key
            mBuckets[bucket].push_back(NaiveMapNode<K, V>(key, value, hash));
            mSize++;
        }

        void insert(const std::pair<K, V> &pair) {
            return insert(pair.first, pair.second);
        }

        void insert_or_assign(const K &key, const V &value) {
            insert(key, value);
        }

        bool erase(const K &key) {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    mBuckets[bucket][i].hash = -1;
                    mSize--;
                    return true;
                }
            }
            return false;
        }

        V &operator[](const K &key) {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return mBuckets[bucket][i].second;
                }
            }
            // insert new key
            mBuckets[bucket].push_back(NaiveMapNode<K, V>(key, V(), hash));
            mSize++;
            return mBuckets[bucket].back().second;
        }

        V &at(const K &key) {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return mBuckets[bucket][i].second;
                }
            }
            throw std::out_of_range("key not found");
        }

        const V &at(const K &key) const {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return mBuckets[bucket][i].second;
                }
            }
            throw std::out_of_range("key not found");
        }

        bool contains(const K &key) const {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return true;
                }
            }
            return false;
        }

        size_t count(const K &key) const {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return 1;
                }
            }
            return 0;
        }

        int size() const {
            return mSize;
        }

        int bucket_count() const {
            return bucketSize;
        }

        int bucket_size(int bucket) const {
            return mBuckets[bucket].size();
        }

        int bucket(const K &key) const {
            int64_t hash = H::hash(key);
            return hash % bucketSize;
        }

        void clear() {
            for (int i = 0; i < bucketSize; i++) {
                mBuckets[i].clear();
            }
            mSize = 0;
        }

        bool empty() const {
            return mSize == 0;
        }

        class iterator {
        public:
            iterator(NaiveMap<K, V, H, bucketSize> *map, int bucket, int index) : mMap(map), mBucket(bucket), mIndex(index) {
                if (mBucket == 0 && mIndex == 0) {
                    mIndex = -1;
                    nextUntilNewValue();
                }
            }

            iterator &operator++() {
                nextUntilNewValue();
                return *this;
            }

            NaiveMapNode<K, V> &operator*() {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return mMap->mBuckets[mBucket][mIndex];
            }

            NaiveMapNode<K, V> *operator->() {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return &mMap->mBuckets[mBucket][mIndex];
            }

            const NaiveMapNode<K, V> &operator*() const {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return mMap->mBuckets[mBucket][mIndex];
            }

            const NaiveMapNode<K, V> *operator->() const {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return &mMap->mBuckets[mBucket][mIndex];
            }

            bool operator==(const iterator &other) const {
                return mMap == other.mMap && mBucket == other.mBucket && mIndex == other.mIndex;
            }

            bool operator!=(const iterator &other) const {
                return !(*this == other);
            }

        private:
            void nextUntilNewValue() {
                while (true) {
                    mIndex++;
                    if (mIndex >= mMap->mBuckets[mBucket].size()) {
                        mBucket++;
                        if (mBucket >= bucketSize) {
                            mBucket = -1;
                            mIndex = -1;
                            return;
                        }
                        mIndex = -1;
                        continue;
                    }
                    if (mMap->mBuckets[mBucket][mIndex].hash == -1) {
                        continue;
                    }
                    break;
                }
            }

        public:
            NaiveMap<K, V, H, bucketSize> *mMap;
            int mBucket;
            int mIndex;
        };

        class const_iterator {
        public:
            const_iterator(const NaiveMap<K, V, H, bucketSize> *map, int bucket, int index) : mMap(map),
                                                                                              mBucket(bucket),
                                                                                              mIndex(index) {
                if (mBucket == 0 && mIndex == 0) {
                    mIndex = -1;
                    nextUntilNewValue();
                }
            }

            const_iterator &operator++() {
                nextUntilNewValue();
                return *this;
            }

            const NaiveMapNode<K, V> &operator*() const {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return mMap->mBuckets[mBucket][mIndex];
            }

            const NaiveMapNode<K, V> *operator->() const {
                assert(mBucket >= 0 && mBucket < mMap->mBuckets.size());
                assert(mIndex >= 0 && mIndex < mMap->mBuckets[mBucket].size());
                assert(mMap->mBuckets[mBucket][mIndex].hash != -1);
                return &mMap->mBuckets[mBucket][mIndex];
            }

            bool operator==(const const_iterator &other) const {
                return mMap == other.mMap && mBucket == other.mBucket && mIndex == other.mIndex;
            }

            bool operator!=(const const_iterator &other) const {
                return !(*this == other);
            }

        private:
            void nextUntilNewValue() {
                while (true) {
                    mIndex++;
                    if (mIndex >= mMap->mBuckets[mBucket].size()) {
                        mBucket++;
                        if (mBucket >= bucketSize) {
                            mBucket = -1;
                            mIndex = -1;
                            return;
                        }
                        mIndex = -1;
                        continue;
                    }
                    if (mMap->mBuckets[mBucket][mIndex].hash == -1) {
                        continue;
                    }
                    break;
                }
            }

        public:
            const NaiveMap<K, V, H, bucketSize> *mMap;
            int mBucket;
            int mIndex;
        };

        iterator begin() {
            return iterator(this, 0, 0);
        }

        iterator end() {
            return iterator(this, -1, -1);
        }

        iterator find(const K &key) {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return iterator(this, bucket, i);
                }
            }
            return end();
        }

        iterator erase(iterator it) {
            mBuckets[it.mBucket][it.mIndex].hash = -1;
            mSize--;
            return ++it;
        }

        const_iterator begin() const {
            return const_iterator(this, 0, 0);
        }

        const_iterator end() const {
            return const_iterator(this, -1, -1);
        }

        const_iterator find(const K &key) const {
            int64_t hash = mHash(key);
            int bucket = hash % bucketSize;
            for (int i = 0; i < mBuckets[bucket].size(); i++) {
                if (mBuckets[bucket][i].hash == hash) {
                    return const_iterator(this, bucket, i);
                }
            }
            return end();
        }

    private:
        std::array<std::vector<NaiveMapNode<K, V>>, 256> mBuckets;
        size_t mSize = 0;
        H mHash;

    };

    namespace Test {

        inline void testNaiveMap() {
            NaiveMap<int, int, std::hash<int>, 256> maps[1000];
            for (int iter = 0; iter < 100; iter++) {
                for (auto map: maps) {
                    for (int i = 0; i < 100; i++) {
                        int v = rand();
                        map[v] = v;
                    }
                    for (auto it = map.begin(); it != map.end(); ++it) {
                        if (it->first != it->second) {
                            abort();
                        }
                        int v = rand() % 2;
                        if (v == 0) {
                            map.erase(it);
                        }
                    }
                }
            }
        }

    }
}

namespace std {

    // structure binding support
    template<typename K, typename V>
    struct tuple_size<CML::NaiveMapNode<K, V>> : std::integral_constant<size_t, 2> {
    };

    template<typename K, typename V>
    struct tuple_element<0, CML::NaiveMapNode<K, V>> {
        using type = K;
    };

    template<typename K, typename V>
    struct tuple_element<1, CML::NaiveMapNode<K, V>> {
        using type = V;
    };

}

#endif