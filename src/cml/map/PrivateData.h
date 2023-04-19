//
// Created by belosth on 14/01/2020.
//

#ifndef CML_PRIVATEDATA_H
#define CML_PRIVATEDATA_H

#include "cml/config.h"
#include "cml/utils/GarbageCollector.h"
#include "cml/utils/PoolAllocator.h"
#include "cml/utils/Complexity.h"

namespace CML {

#define CML_PRIVATEDATA_MAX_FUNCTIONS 32
#define CML_PRIVATEDATA_NOTALLOCATED (PrivateDataStructure*)0
#define CML_PRIVATEDATA_FREED (PrivateDataStructure*)1
#define CML_PRIVATEDATA_USEPOOL 1
#define CML_ENABLE_PRIVATEDATA_FREE_REASON ENABLE_ASSERTTHROW

    class PrivateDataStructure : public Garbage {

    public:
        virtual ~PrivateDataStructure() = default;

    };

    template<class Test, class Base>
    struct AssertSameOrDerivedFrom {
        AssertSameOrDerivedFrom() { UNUSED(&constraints); }
    public:
        static void constraints() {
            Test *pd = 0;
            Base *pb = pd;
            UNUSED(pb);
        }
    };

    class PrivateDataInstance {

        friend class PrivateDataContext;
        friend class PrivateData;
    public:
        PrivateDataInstance() : mId(-1) {
            mPoolAllocator = std::make_shared<PoolAllocator>(256);
        }

    private:
        PrivateDataInstance(int id) : mId(id) {
            mPoolAllocator = std::make_shared<PoolAllocator>(256);
        }

        int mId;
        std::shared_ptr<PoolAllocator> mPoolAllocator;

    };

    class PrivateDataContext {

        friend class PrivateData;

    public:
        PrivateDataContext() {
            mCurrentId = 0;
        }

        PrivateDataInstance createInstance() {
            assertThrow(mCurrentId <= CML_PRIVATEDATA_MAX_FUNCTIONS, "Too many functions inside private data");
            PrivateDataInstance instance(mCurrentId++);
            mAllInstances.emplace_back(instance);
            return instance;
        }

    private:
        Atomic<int> mCurrentId;
        List<PrivateDataInstance> mAllInstances;

    };

    class PrivateProxyPoolGarbage : public Garbage {

    public:
        PrivateProxyPoolGarbage(PoolAllocator *poolAllocator, PrivateDataStructure *data) : mPoolAllocator(poolAllocator), mData(data) {

        }

        ~PrivateProxyPoolGarbage() {
            mData->~PrivateDataStructure();
            mPoolAllocator->deallocate(mData);
        }

    private:
        PoolAllocator *mPoolAllocator;
        PrivateDataStructure *mData;


    };

    class PrivateData {

    public:
        PrivateData() {
            for (int i = 0; i < CML_PRIVATEDATA_MAX_FUNCTIONS; i++) {
                mFunctions[i] = CML_PRIVATEDATA_NOTALLOCATED;
            }
        }

        ~PrivateData() {
            for (int i = 0; i < CML_PRIVATEDATA_MAX_FUNCTIONS; i++) {
                if (mFunctions[i] != CML_PRIVATEDATA_NOTALLOCATED && mFunctions[i] != CML_PRIVATEDATA_FREED) {
#if CML_PRIVATEDATA_USEPOOL
                    abort();
#else
                    delete mFunctions[i];
#endif
                }
            }
        }

        void reset() {
            for (int i = 0; i < CML_PRIVATEDATA_MAX_FUNCTIONS; i++) {
                mFunctions[i] = CML_PRIVATEDATA_NOTALLOCATED;
            }
        }

        void freeAll(const PrivateDataContext &context, GarbageCollector &garbageCollector) {
            signalMethodStart("PrivateData::freeAll");
            for (int i = 0; i < context.mAllInstances.size(); i++) {
                free(context.mAllInstances[i], garbageCollector, "Free all");
            }
        }

        template <typename T> T* get(const PrivateDataInstance& instance) {
            signalMethodStart("PrivateData::get");
            AssertSameOrDerivedFrom<T, PrivateDataStructure>();
#if CML_ENABLE_PRIVATEDATA_FREE_REASON
            assertThrow(mFunctions[instance.mId] != CML_PRIVATEDATA_FREED, "This private data has been freed previously : " + mReasons[instance.mId]);
#else
            assertThrow(mFunctions[instance.mId] != CML_PRIVATEDATA_FREED, "This private data has been freed previously");
#endif
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_NOTALLOCATED) {
#if CML_PRIVATEDATA_USEPOOL
                mFunctions[instance.mId] = static_cast<PrivateDataStructure *>(instance.mPoolAllocator->allocate(sizeof(T)));
                new(mFunctions[instance.mId]) T;
#else
                mFunctions[instance.mId] = new T();
#endif
            }
            return (T*)mFunctions[instance.mId];
        }

        template <typename T> T* get(const PrivateDataInstance& instance, bool &isNew) {
            signalMethodStart("PrivateData::get");
            AssertSameOrDerivedFrom<T, PrivateDataStructure>();
#if CML_ENABLE_PRIVATEDATA_FREE_REASON
            assertThrow(mFunctions[instance.mId] != CML_PRIVATEDATA_FREED, "This private data has been freed previously : " + mReasons[instance.mId]);
#else
            assertThrow(mFunctions[instance.mId] != CML_PRIVATEDATA_FREED, "This private data has been freed previously");
#endif
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_NOTALLOCATED) {
#if CML_PRIVATEDATA_USEPOOL
                mFunctions[instance.mId] = static_cast<PrivateDataStructure *>(instance.mPoolAllocator->allocate(sizeof(T)));
                new(mFunctions[instance.mId]) T;
#else
                mFunctions[instance.mId] = new T();
#endif
                isNew = true;
            } else {
                isNew = false;
            }
            return (T*)mFunctions[instance.mId];
        }

        template <typename T> T* unsafe_get(const PrivateDataInstance& instance) {
            signalMethodStart("PrivateData::unsafe_get");
            return (T*)mFunctions[instance.mId];
        }

        inline bool have(const PrivateDataInstance& instance) {
            signalMethodStart("PrivateData::have");
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_NOTALLOCATED) return false;
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_FREED) return false;
            return true;
        }

        inline void free(const PrivateDataInstance& instance, const std::string& reason) {
            signalMethodStart("PrivateData::free");
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_NOTALLOCATED) {
                mFunctions[instance.mId] = CML_PRIVATEDATA_FREED;
                return;
            }
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_FREED) {
                return;
            }
#if CML_PRIVATEDATA_USEPOOL
            mFunctions[instance.mId]->~PrivateDataStructure();
            instance.mPoolAllocator->deallocate(mFunctions[instance.mId]);
#else
            delete mFunctions[instance.mId];
#endif
            mFunctions[instance.mId] = CML_PRIVATEDATA_FREED;
#if CML_ENABLE_PRIVATEDATA_FREE_REASON
            mReasons[instance.mId] = reason;
#endif
        }

        inline void free(PrivateDataInstance instance, GarbageCollector &garbageCollector, std::string reason) {
            signalMethodStart("PrivateData::free");
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_NOTALLOCATED) {
                mFunctions[instance.mId] = CML_PRIVATEDATA_FREED;
                return;
            }
            if (mFunctions[instance.mId] == CML_PRIVATEDATA_FREED) {
                return;
            }
#if CML_PRIVATEDATA_USEPOOL
            garbageCollector.erase(new PrivateProxyPoolGarbage(instance.mPoolAllocator.get(), mFunctions[instance.mId]));
#else
            garbageCollector.erase( mFunctions[instance.mId]);
#endif
            mFunctions[instance.mId] = CML_PRIVATEDATA_FREED;
#if CML_ENABLE_PRIVATEDATA_FREE_REASON
            mReasons[instance.mId] = reason;
#endif
        }

    private:
       // HashMap<void*, void*> mMap;
       PrivateDataStructure* mFunctions[CML_PRIVATEDATA_MAX_FUNCTIONS];
#if CML_ENABLE_PRIVATEDATA_FREE_REASON
       std::string mReasons[CML_PRIVATEDATA_MAX_FUNCTIONS];
#endif

    };

}

#endif //CML_PRIVATEDATA_H
