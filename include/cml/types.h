#ifndef CML_TYPES_H
#define CML_TYPES_H

#include <filesystem>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <dirent.h>
#endif

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

// #define SPP_DEFAULT_ALLOCATOR Eigen::aligned_allocator
#include <sparsepp/spp.h>
#if CML_USE_GOOGLE_HASH
#include <sparsehash/dense_hash_map>
#include <sparsehash/dense_hash_set>
#endif

#include <vector>
#include <list>
#include <atomic>
#include <mutex>
#include <set>
#include <condition_variable>
#include <numeric>

#include "types/Optional.h"
#include "utils/Logger.h"
#include "fastmath.h"

#define UNUSED(x) (void)(x)

#define CML_PIXEL_GET_IS_CENTER 0



namespace CML {

#ifdef WIN32
    inline void usleep(__int64 usec)
    {
        HANDLE timer;
        LARGE_INTEGER ft;

        ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

        timer = CreateWaitableTimer(NULL, TRUE, NULL);
        SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
        WaitForSingleObject(timer, INFINITE);
        CloseHandle(timer);
    }
#else
    inline void usleep(__int64_t usec) {
        ::usleep(usec);
    }
#endif

    inline float memoryUsage() {
#ifdef linux
        FILE *file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];

        while (fgets(line, 128, file) != NULL) {
            if (strncmp(line, "VmRSS:", 6) == 0) {

                {
                    int i = strlen(line);
                    const char* p = line;
                    while (*p <'0' || *p > '9') p++;
                    line[i-3] = '\0';
                    i = atoi(p);
                    result = i;
                }

                break;

            }
        }
        fclose(file);

        float memoryUsage = (float)result * 0.001f;
        return memoryUsage;
#else
        return 0;
#endif
    }

    inline bool hasEnding (std::string const &fullString, std::string const &ending) {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }

    inline std::vector<std::string> listDirectory(std::string path, std::string ext = "") {

        std::vector<std::string> result;

        for (const auto & entry : std::filesystem::directory_iterator(path)) {
            result.emplace_back(entry.path().filename().string());
        }

        return result;

    }

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#define UNUSED(x) (void)(x)

    inline void printAndAbort(const char* expression, const char* file, int line, std::string msg) {

        std::string what = (std::string)"Assertion " + expression + " failed, file " + file + " line " + std::to_string(line);

        logger.fatal(what);
        logger.fatal(msg);

        asm("int $3");
         abort();

    }

#ifndef ENABLE_ASSERTTHROW_ON_RELEASE
#define ENABLE_ASSERTTHROW_ON_RELEASE 0
#endif

#ifndef NDEBUG
#define ENABLE_ASSERTTHROW 1
#else
#if ENABLE_ASSERTTHROW_ON_RELEASE
#define ENABLE_ASSERTTHROW 1
#else
#define ENABLE_ASSERTTHROW 0
#endif
#endif

#if ENABLE_ASSERTTHROW
#define assertThrow(EXPRESSION, MSG) if (!(EXPRESSION)) printAndAbort(#EXPRESSION, __FILE__, __LINE__, MSG)
#else
#define assertThrow(EXPRESSION, MSG)
#endif

    template <typename T> inline void assertDeterministic(const std::string &msg, T value) {
        //logger.raw(std::to_string(value) + "  |  " + msg + "\n");
    }

    inline void assertDeterministic(const std::string &msg) {
        //logger.raw(msg + "\n");
    }

    inline void setThreadName(std::string name) {
        pthread_setname_np(pthread_self(), name.c_str());
    }

    struct Hasher;

    template <typename T, typename U>
            using Pair = std::pair<T, U>;

    template <typename T>
            using List = std::vector<T, Eigen::aligned_allocator<T>>;

    template <typename T>
            using LinkedList = std::list<T, Eigen::aligned_allocator<T>>;

    template <typename T, typename H = Hasher>
            using Set = spp::sparse_hash_set<T, H, std::equal_to<T>>;

    template <typename T, typename C=std::less<T>>
            using OrderedSet = std::set<T, C, Eigen::aligned_allocator<T>>;

    template <typename T, typename U, typename H = Hasher>
            using HashMap = spp::sparse_hash_map<T, U, H, std::equal_to<T>>;

#if CML_USE_GOOGLE_HASH
    template <typename T, typename U, typename H = std::hash<T>>
        using DenseHashMap = google::dense_hash_map<T, U, H, std::equal_to<T>>;
#else
    template <typename T, typename U, typename H = Hasher>
        using DenseHashMap = spp::sparse_hash_map<T, U, H, std::equal_to<T>>;
#endif


    template <typename T>
            using Atomic = std::atomic<T>;

    using Mutex = std::mutex;
    using LockGuard = std::lock_guard<std::mutex>;
    using UniqueLock = std::unique_lock<std::mutex>;
    using ConditionVariable = std::condition_variable;

    template <typename T, int C> class Queue
    {
    public:
        Queue() {
            mSize = 0;
            mPushIndex = 0;
            mPopIndex = 0;
        }

        EIGEN_STRONG_INLINE T *getPushElement() {
            if (mIsDestroyed) {
                return &mElements[mPushIndex];
            }
            UniqueLock lock(mMutex);
            while (mSize == C) {
                mConditionVariable.wait_for(lock, std::chrono::milliseconds(10));
                if (mIsDestroyed) {
                    return &mElements[mPushIndex];
                }
            }
            T *element = &mElements[mPushIndex];
            return element;
        }

        EIGEN_STRONG_INLINE void notifyPush() {
            mPushIndex = (mPushIndex + 1) % C;
            mSize++;
            mConditionVariable.notify_all();
        }

        EIGEN_STRONG_INLINE T *getPopElement()
        {
            if (mIsDestroyed) {
                mElements[mPopIndex] = T();
                return &mElements[mPopIndex];
            }
            UniqueLock lock(mMutex);
            while (mSize == 0) {
                mConditionVariable.wait_for(lock, std::chrono::milliseconds(10));
                if (mIsDestroyed) {
                    mElements[mPopIndex] = T();
                    return &mElements[mPopIndex];
                }
            }
            T *element = &mElements[mPopIndex];
            return element;
        }

        EIGEN_STRONG_INLINE void notifyPop() {
            mPopIndex = (mPopIndex + 1) % C;
            mSize--;
            mConditionVariable.notify_all();
        }

        EIGEN_STRONG_INLINE void destroy() {
            mIsDestroyed = true;
            mConditionVariable.notify_all();
        }

        EIGEN_STRONG_INLINE bool isDestroyed() {
            return mIsDestroyed;
        }

        EIGEN_STRONG_INLINE int getCurrentSize() {
            return mSize;
        }

    private:
        T mElements[C];
        Atomic<int> mSize;
        int mPushIndex, mPopIndex;
        Mutex mMutex;
        ConditionVariable mConditionVariable;
        Atomic<bool> mIsDestroyed = false;
    };

    template <typename T> class Window {

    public:
        void add(const T &t) {
            mList.push_front(t);
        }

        T accumulate(size_t size) {
            mSize = size;
            while (mList.size() > mSize) {
                mList.pop_back();
            }
            assertThrow(mList.size() <= mSize, "Implementation problem of window");
            T result(0);
            bool isFirst = true;
            for (const T &t : mList) {
                if (isFirst) {
                    result = t;
                    isFirst = false;
                } else {
                    result += t;
                }
            }
            return result;
        }

    private:
        size_t mSize;
        LinkedList<T> mList;

    };


    template <typename T> List<T> fromStd(std::vector<T> a) {
        return List<T>(a.begin(), a.end());
    }

    template <typename T> Atomic<T>& operator+= (Atomic<T>& atomicFloat, T increment)
    {
        T oldValue;
        T newValue;

        do
        {
            oldValue = atomicFloat.load (std::memory_order_relaxed);
            newValue = oldValue + increment;
        } while (! atomicFloat.compare_exchange_weak (oldValue, newValue,
                                                      std::memory_order_release,
                                                      std::memory_order_relaxed));
        return atomicFloat;
    }

#define CML_SCALAR_TYPE double
#define CML_GL_SCALAR GL_DOUBLE
    typedef CML_SCALAR_TYPE scalar_t;

    // From : https://github.com/xiezhq-hermann/atan_lookup/blob/master/atan.cpp
    const float LUT[102] = {
            0,           0.0099996664, 0.019997334, 0.029991005, 0.039978687,
            0.049958397, 0.059928156,  0.069885999, 0.079829983, 0.089758173,
            0.099668652, 0.10955953,   0.11942893,  0.12927501,  0.13909595,
            0.14888994,  0.15865526,   0.16839015,  0.17809294,  0.18776195,
            0.19739556,  0.20699219,   0.21655031,  0.22606839,  0.23554498,
            0.24497867,  0.25436807,   0.26371184,  0.27300870,  0.28225741,
            0.29145679,  0.30060568,   0.30970293,  0.31874755,  0.32773849,
            0.33667481,  0.34555557,   0.35437992,  0.36314702,  0.37185606,
            0.38050637,  0.38909724,   0.39762798,  0.40609807,  0.41450688,
            0.42285392,  0.43113875,   0.43936089,  0.44751999,  0.45561564,
            0.46364760,  0.47161558,   0.47951928,  0.48735857,  0.49513325,
            0.50284320,  0.51048833,   0.51806855,  0.52558380,  0.53303409,
            0.54041952,  0.54774004,   0.55499572,  0.56218672,  0.56931317,
            0.57637525,  0.58337301,   0.59030676,  0.59717667,  0.60398299,
            0.61072594,  0.61740589,   0.62402308,  0.63057774,  0.63707036,
            0.64350110,  0.64987046,   0.65617871,  0.66242629,  0.66861355,
            0.67474097,  0.68080884,   0.68681765,  0.69276786,  0.69865984,
            0.70449406,  0.71027100,   0.71599114,  0.72165483,  0.72726268,
            0.73281509,  0.73831260,   0.74375558,  0.74914461,  0.75448018,
            0.75976276,  0.76499283,   0.77017093,  0.77529752,  0.78037310,
            0.78539819,  0.79037325};

    EIGEN_STRONG_INLINE float atan_single(float x) {
        /*
        Linear interpolation is used for higher accuracy
        */
        if (x >= 0) {
            if (x <= 1) {
                int index = round(x * 100);
                return (LUT[index] + (x * 100 - index) * (LUT[index + 1] - LUT[index]));
            } else {
                float re_x = 1 / x;
                int index = round(re_x * 100);
                return (M_PI_2 - (LUT[index] + (re_x * 100 - index) * (LUT[index + 1] - LUT[index])));
                // No recursive is better here
            }
        } else {
            if (x >= -1) {
                float abs_x = -x;
                int index = round(abs_x * 100);
                return -(LUT[index] + (abs_x * 100 - index) * (LUT[index + 1] - LUT[index]));
            } else {
                float re_x = 1 / (-x);
                int index = round(re_x * 100);
                return (LUT[index] + (re_x * 100 - index) * (LUT[index+1] - LUT[index])) - M_PI_2;
            }
        }
    }

    inline scalar_t cos(scalar_t v) { return std::cos(v); }
    inline scalar_t sin(scalar_t v) { return std::sin(v); }
    inline scalar_t tan(scalar_t v) { return std::tan(v); }
    inline scalar_t sqrt(scalar_t v) { return std::sqrt(v); }

    inline scalar_t acos(scalar_t v) { return std::acos(v); }
    inline scalar_t asin(scalar_t v) { return std::asin(v); }
    inline scalar_t atan(scalar_t v) { return std::atan(v); }

    inline scalar_t atan_fast(scalar_t v) { return atan_single(v); }

    template <typename T> int sign(T val) {
        return (T(0) < val) - (val < T(0));
    }

    const scalar_t PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664709384460955058223172535940812848111;

    template <int r, int c> using Matrix = Eigen::Matrix<scalar_t, r, c>;
    template <int r> using Vector = Eigen::Matrix<scalar_t, r, 1>;

    template <int r, int c> using Matrixd = Eigen::Matrix<double, r, c>;
    template <int r> using Vectord = Eigen::Matrix<double, r, 1>;

    template <int r, int c> using Matrixf = Eigen::Matrix<float, r, c>;
    template <int r> using Vectorf = Eigen::Matrix<float, r, 1>;

    template <int r, int c> using Matrixi = Eigen::Matrix<int, r, c>;
    template <int r> using Vectori = Eigen::Matrix<int, r, 1>;

    using Vector2 = Matrix<2, 1>;
    using Vector3 = Matrix<3, 1>;
    using Vector4 = Matrix<4, 1>;
    using Vector5 = Matrix<5, 1>;
    using Vector6 = Matrix<6, 1>;
    using Vector7 = Matrix<7, 1>;
    using Vector8 = Matrix<8, 1>;
    using Vector9 = Matrix<9, 1>;

    using Matrix22 = Matrix<2, 2>;
    using Matrix33 = Matrix<3, 3>;
    using Matrix44 = Matrix<4, 4>;
    using Matrix23 = Matrix<2, 3>;
    using Matrix32 = Matrix<3, 2>;
    using Matrix34 = Matrix<3, 4>;
    using Matrix43 = Matrix<4, 3>;

    using Vector2d = Matrixd<2, 1>;
    using Vector3d = Matrixd<3, 1>;
    using Vector4d = Matrixd<4, 1>;
    using Vector5d = Matrixd<5, 1>;
    using Vector6d = Matrixd<6, 1>;
    using Vector7d = Matrixd<7, 1>;
    using Vector8d = Matrixd<8, 1>;
    using Vector9d = Matrixd<9, 1>;

    using Matrix22d = Matrixd<2, 2>;
    using Matrix33d = Matrixd<3, 3>;
    using Matrix44d = Matrixd<4, 4>;
    using Matrix23d = Matrixd<2, 3>;
    using Matrix32d = Matrixd<3, 2>;
    using Matrix34d = Matrixd<3, 4>;
    using Matrix43d = Matrixd<4, 3>;


    using Vector2f = Matrixf<2, 1>;
    using Vector3f = Matrixf<3, 1>;
    using Vector4f = Matrixf<4, 1>;
    using Vector5f = Matrixf<5, 1>;
    using Vector6f = Matrixf<6, 1>;
    using Vector7f = Matrixf<7, 1>;
    using Vector8f = Matrixf<8, 1>;
    using Vector9f = Matrixf<9, 1>;

    using Matrix22f = Matrixf<2, 2>;
    using Matrix33f = Matrixf<3, 3>;
    using Matrix44f = Matrixf<4, 4>;
    using Matrix23f = Matrixf<2, 3>;
    using Matrix32f = Matrixf<3, 2>;
    using Matrix34f = Matrixf<3, 4>;
    using Matrix43f = Matrixf<4, 3>;

    using Vector2i = Matrixi<2, 1>;
    using Vector3i = Matrixi<3, 1>;
    using Vector4i = Matrixi<4, 1>;
    using Vector5i = Matrixi<5, 1>;
    using Vector6i = Matrixi<6, 1>;
    using Vector7i = Matrixi<7, 1>;
    using Vector8i = Matrixi<8, 1>;
    using Vector9i = Matrixi<9, 1>;

    using Matrix22i = Matrixi<2, 2>;
    using Matrix33i = Matrixi<3, 3>;
    using Matrix44i = Matrixi<4, 4>;
    using Matrix23i = Matrixi<2, 3>;
    using Matrix32i = Matrixi<3, 2>;
    using Matrix34i = Matrixi<3, 4>;
    using Matrix43i = Matrixi<4, 3>;

    inline bool almostEqual(scalar_t a, scalar_t b) {
        return fabs(a - b) < (scalar_t)0.001;
    }

    template <int r, int c> bool almostEqual(const Matrix<r, c> &a, const Matrix<r, c> &b) {
        return (a - b).norm() < (scalar_t)0.001;
    }


    const int Dynamic = Eigen::Dynamic;

    const bool NonNullable = false;
    const bool Nullable = true;

#define CML_PTR_MAGICNUMBER ENABLE_ASSERTTHROW

    template <typename T, bool isNullable> class Ptr {

    public:
        Ptr() {
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                assertThrow(false, "Call non nullable ptr default constructor");
            }
#endif
#if CML_PTR_MAGICNUMBER
            setupMagicNumber();
#endif
        }

        Ptr(T *p) : mP(p) {
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                validPtrAssert("");
            }
#endif
#if CML_PTR_MAGICNUMBER
            setupMagicNumber();
#endif
        }

        template <bool otherIsNullable> Ptr(const Ptr<T, otherIsNullable> &ptr) : mP(ptr.p()) {
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                ptr.validPtrAssert("");
            }
#endif
#if CML_PTR_MAGICNUMBER
            setupMagicNumber();
#endif
        }

        inline Ptr<T, isNullable>& operator=(T *p) {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
#endif
            mP = p;
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                validPtrAssert("");
            }
#endif
            return *this;
        }

        template <bool otherIsNullable> inline Ptr<T, isNullable>& operator=(const Ptr<T, otherIsNullable> &ptr) {
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                ptr.validPtrAssert("");
            }
#endif
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
            ptr.magicNumberAssert();
#endif
            mP = ptr.p();
            return *this;
        }

        inline bool isNull() const {
            return mP == nullptr;
        }

        inline bool isNotNull() const {
            return mP != nullptr;
        }

        inline T* p() const {
#if ENABLE_ASSERTTHROW
            if (!isNullable) {
                validPtrAssert("Memory corruption");
            }
#endif
            return mP;
        }

        inline T& operator*() {
#if ENABLE_ASSERTTHROW
            validPtrAssert("");
#endif
            return mP;
        }

        inline const T& operator*() const {
#if ENABLE_ASSERTTHROW
            validPtrAssert("");
#endif
            return mP;
        }

        inline T* operator->() const {
#if ENABLE_ASSERTTHROW
            validPtrAssert("");
#endif
            return mP;
        }

        template <bool otherIsNullable> inline bool operator==(const Ptr<T, otherIsNullable> &other) const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
            other.magicNumberAssert();
#endif
            return mP == other.p();
        }

        template <bool otherIsNullable> inline bool operator<(const Ptr<T, otherIsNullable> &other) const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
            other.magicNumberAssert();
#endif
            return mP < other.p();
        }

        template <bool otherIsNullable> inline bool operator>(const Ptr<T, otherIsNullable> &other) const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
            other.magicNumberAssert();
#endif
            return mP > other.p();
        }

        inline bool operator==(T *other) const {
            return mP == other;
        }

        template <bool otherIsNullable> inline bool operator!=(const Ptr<T, otherIsNullable> &other) const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
            other.magicNumberAssert();
#endif
            return mP != other.p();
        }

        inline bool operator!=(T *other) const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
#endif
            return mP != other;
        }

        Ptr<T, Nullable> nullable() const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
#endif
            return Ptr<T, Nullable>(mP);
        }

        Ptr<T, NonNullable> nonNullable() const {
#if CML_PTR_MAGICNUMBER
            magicNumberAssert();
#endif
            return Ptr<T, NonNullable>(mP);
        }

        inline void setupMagicNumber() {
            // a = rand();
#if CML_PTR_MAGICNUMBER
            a = ((unsigned char*)(&mP))[6];
            b = a + 1;
#endif
        }

        inline void magicNumberAssert() const {
#if CML_PTR_MAGICNUMBER
            unsigned char btruth = a + 1;
            if (b != btruth) {
                logger.fatal("Magic number not equal. Memory corruption !");
                abort();
            }
#endif
        }

        inline void validPtrAssert(const std::string& more) const {
            assertThrow(mP != nullptr, "Null pointer " + more);
            // In the Unix world, it is typical to reserve all the address space below 0x8000 for null values.
            assertThrow((void*)mP > (void*)0x8000, "Invalid pointer " + more);
        }

    private:
#if CML_PTR_MAGICNUMBER
        unsigned char a;
        unsigned char b;
#endif

        T* mP = nullptr;
    };

    class AbstractFunction;

    class InternalCalibration;

    class ColorRGBA {

    public:
        EIGEN_STRONG_INLINE ColorRGBA() {
            mColor[0] = 0;
            mColor[1] = 0;
            mColor[2] = 0;
            mColor[3] = 0;
        }

        EIGEN_STRONG_INLINE ColorRGBA(uint8_t r, uint8_t g, uint8_t b) {
            mColor[0] = r;
            mColor[1] = g;
            mColor[2] = b;
            mColor[3] = 1;
        }

        EIGEN_STRONG_INLINE ColorRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
            mColor[0] = r;
            mColor[1] = g;
            mColor[2] = b;
            mColor[3] = 1;
        }

        EIGEN_STRONG_INLINE ColorRGBA(uint8_t v) {
            mColor[0] = v;
            mColor[1] = v;
            mColor[2] = v;
            mColor[3] = 1;
        }

        template <typename U> ColorRGBA(U r, U g, U b) {
            mColor[0] = (uint8_t)r;
            mColor[1] = (uint8_t)g;
            mColor[2] = (uint8_t)b;
            mColor[3] = 1;
        }

        template <typename U> ColorRGBA(U v) {
            mColor[0] = (uint8_t)v;
            mColor[1] = (uint8_t)v;
            mColor[2] = (uint8_t)v;
            mColor[3] = 1;
        }

        template <typename U> ColorRGBA(std::complex<U> v) {
            mColor[0] = v.real();
            mColor[1] = v.imag();
            mColor[2] = 0;
            mColor[3] = 1;
        }

        EIGEN_STRONG_INLINE uint8_t &operator()(int x) {
            return mColor[x];
        }

        EIGEN_STRONG_INLINE uint8_t operator()(int x) const {
            return mColor[x];
        }

        EIGEN_STRONG_INLINE ColorRGBA operator+(ColorRGBA other) const {
            return ColorRGBA(mColor[0] + other.mColor[0], mColor[1] + other.mColor[1], mColor[2] + other.mColor[2]);
        }

        EIGEN_STRONG_INLINE ColorRGBA operator*(scalar_t v) const {
            return ColorRGBA((float) mColor[0] * v, (float) mColor[1] * v, (float) mColor[2] * v);
        }

        EIGEN_STRONG_INLINE ColorRGBA operator/(scalar_t v) const {
            return ColorRGBA((float) mColor[0] / v, (float) mColor[1] / v, (float) mColor[2] / v);
        }

        EIGEN_STRONG_INLINE float gray() const {
            return weightedGray<0>() + weightedGray<1>() + weightedGray<2>();
        }

        EIGEN_STRONG_INLINE uint8_t r() const {
            return mColor[0];
        }

        EIGEN_STRONG_INLINE uint8_t g() const {
            return mColor[1];
        }

        EIGEN_STRONG_INLINE uint8_t b() const {
            return mColor[2];
        }

        EIGEN_STRONG_INLINE uint8_t a() const {
            return mColor[3];
        }

        EIGEN_STRONG_INLINE uint8_t &r() {
            return mColor[0];
        }

        EIGEN_STRONG_INLINE uint8_t &g() {
            return mColor[1];
        }

        EIGEN_STRONG_INLINE uint8_t &b() {
            return mColor[2];
        }

        EIGEN_STRONG_INLINE uint8_t &a() {
            return mColor[3];
        }

        EIGEN_STRONG_INLINE Eigen::Matrix<uint8_t, 3, 1> eigen() const {
            return Eigen::Matrix<uint8_t, 3, 1>(mColor[0], mColor[1], mColor[2]);
        }

    protected:
        float gamma(float value) const { // Value between 0 and 1
            if (value < 0.04045) {
                return value / 12.92f;
            } else {
                return powf((value + 0.055f) / 1.055f, 2.4f);
            }
        }

        template<int i> float weightedGray() const {
            float v = (float)mColor[i] / 255.0f;
            float g = gamma(v);
            float w = 0;
            if (i == 0) {
                w = 0.2126;
            }
            if (i == 1) {
                w = 0.7152;
            }
            if (i == 2) {
                w = 0.0722;
            }
            return w * g * 255.0f;
        }

    private:
        uint8_t mColor[4];

    };

    class ColorRGB {

    public:
        ColorRGB() {
            mColor[0] = 0;
            mColor[1] = 0;
            mColor[2] = 0;
        }

        ColorRGB(ColorRGBA color) {
            mColor[0] = color.r();
            mColor[1] = color.g();
            mColor[2] = color.b();
        }

    private:
        uint8_t mColor[3];

    };

    class GrayLookupTable;

    template <typename T> class AbstractROArray2D;

    template <typename FROM, typename TO> class AbstractProxyArray2D;

    template<typename T> class Array2D;

    template <typename T> class MatrixPyramid;

    using Image = Array2D<ColorRGBA>;

    using GrayImage = Array2D<unsigned char>;

    using GradientImage = Array2D<Vector3f>;

    using FloatImage = Array2D<float>;

    template <typename T> class MatrixPool;

    class CaptureImage;

    /// Rotation //////////////////////////////////////////
    template <int N> class Rotation;

    class AxisAngle;
    class AxisAngleMagnitude;
    class Quaternion;

    ///////////////////////////////////////////////////////

    class Camera;

    class WorldPoint;

    class Frame;
    using PFrame = Ptr<Frame, NonNullable>;
    using OptPFrame = Ptr<Frame, Nullable>;

    class MapPoint;
    using PPoint = Ptr<MapPoint, NonNullable>;
    using OptPPoint = Ptr<MapPoint, Nullable>;

    class Map;

    extern Atomic<size_t> mHashCounter;

    class DeterministicallyHashable {

    public:
        inline DeterministicallyHashable() {
            mHash = mHashCounter++;
        }

        inline size_t hash() const {
            return mHash;
        }

    private:
        size_t mHash;

    };

    class Hasher
    {
    public:
        size_t operator()(PFrame pFrame) const;
        size_t operator()(PPoint pPoint) const;
        inline size_t operator()(const DeterministicallyHashable &obj) const {
            return obj.hash();
        }
        inline size_t operator()(const DeterministicallyHashable *obj) const {
            return obj->hash();
        }
        inline size_t operator()(size_t v) const {
            return v;
        }
        inline size_t operator()(const std::string &v) const {
            return std::hash<std::string>()(v);
        }
    };

    class Comparator {
    public:
        bool operator() (PPoint pPointA, PPoint pPointB) const;
        bool operator() (PFrame pFrameA, PFrame pFrameB) const;
        inline size_t operator()(const DeterministicallyHashable &objA, const DeterministicallyHashable &objB) const {
            return objA.hash() > objB.hash();
        }
        inline size_t operator()(const DeterministicallyHashable *objA, const DeterministicallyHashable *objB) const {
            return objA->hash() > objB->hash();
        }
        inline size_t operator()(size_t a, size_t b) const {
            return a > b;
        }
    };



    template <typename T> using FrameHashMap = HashMap<PFrame, T, Hasher>;
    template <typename T> using PointHashMap = HashMap<PPoint, T, Hasher>;

    using FrameSet = Set<PFrame, Hasher>;
    using PointSet = Set<PPoint, Hasher>;

    class PrivateData;

    typedef enum {
        NOT_INITIALIZED, OK, LOST
    } InitializationState;

    inline std::string toString(InitializationState state) {
        switch (state) {
            case NOT_INITIALIZED:
                return "Not initialized";
            case OK:
                return "Ok";
            case LOST:
                return "Lost";
        }
    }

    template<typename T> std::string matToString(const T & mat){
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }

    typedef enum {
        INDIRECT = 0, DIRECT = 1
    } MapPointType;

#define SCALEFACTOR 2

    class UndistortedVector2d;
    class DistortedVector2d;
    class NormalizedVector2d;

    class UndistortedVector2d : public Vector2  {

    public:
        UndistortedVector2d() : Vector2() {

        }

        explicit UndistortedVector2d(scalar_t x, scalar_t y) : Vector2(x, y) {

        }

        explicit UndistortedVector2d(Vector2 p) : Vector2(p) {

        }

        UndistortedVector2d(const DistortedVector2d &p) = delete;

        UndistortedVector2d(const NormalizedVector2d &p) = delete;

    };

    class DistortedVector2d : public Vector2 {

    public:
        DistortedVector2d() : Vector2() {

        }

        explicit DistortedVector2d(scalar_t x, scalar_t y) : Vector2(x, y) {

        }

        explicit DistortedVector2d(const Vector2 &p) : Vector2(p) {

        }

        DistortedVector2d(const UndistortedVector2d &p) = delete;

        DistortedVector2d(const NormalizedVector2d &p) = delete;

    };

    class NormalizedVector2d : public Vector2 {

    public:
        NormalizedVector2d() : Vector2() {

        }

        explicit NormalizedVector2d(scalar_t x, scalar_t y) : Vector2(x, y) {

        }

        explicit NormalizedVector2d(Vector2 p) : Vector2(p) {

        }

        NormalizedVector2d(const DistortedVector2d &p) = delete;

        NormalizedVector2d(const UndistortedVector2d &p) = delete;

    };

    class Corner {

    public:
        Corner() : mX(0), mY(0) {
        }

        Corner(const DistortedVector2d &p) : mX(p.x()), mY(p.y()) {

        }

        EIGEN_STRONG_INLINE DistortedVector2d point(int level) const {
            Vector2 result(mX, mY);
            for (int i = 0; i < level; i++) {
                result = result / SCALEFACTOR;
            }
            return DistortedVector2d(result);
        }

        EIGEN_STRONG_INLINE scalar_t processScaleFactorFromLevel() const {
            return std::pow(mScaleFactor, level());
        }

        EIGEN_STRONG_INLINE scalar_t processScaleFactorFromLevel(int n) const {
            return std::pow(mScaleFactor, n);
        }

        EIGEN_STRONG_INLINE DistortedVector2d point0() const {
             return DistortedVector2d(mX, mY);
        }

        EIGEN_STRONG_INLINE void scalePoint(scalar_t s) {
            mX *= s;
            mY *= s;
        }

        EIGEN_STRONG_INLINE void scalePoint(scalar_t sx, scalar_t sy) {
            mX *= sx;
            mY *= sy;
        }

        EIGEN_STRONG_INLINE void padPoint(scalar_t x, scalar_t y) {
            mX += x;
            mY += y;
        }

        EIGEN_STRONG_INLINE int level() const {
            return mLevel;
        }

        EIGEN_STRONG_INLINE void setLevel(int level) {
            mLevel = level;
        }

        EIGEN_STRONG_INLINE int size() const {
            return mSize;
        }

        EIGEN_STRONG_INLINE void setSize(int size) {
            mSize = size;
        }

        EIGEN_STRONG_INLINE scalar_t angle() const {
            return mAngle;
        }

        EIGEN_STRONG_INLINE void setAngle(scalar_t angle) {
            mAngle = angle;
        }

        EIGEN_STRONG_INLINE scalar_t response() const {
            return mResponse;
        }

        EIGEN_STRONG_INLINE void setResponse(scalar_t response) {
            mResponse = response;
        }

        EIGEN_STRONG_INLINE double x() const {
            return mX;
        }

        EIGEN_STRONG_INLINE double y() const {
            return mY;
        }

        EIGEN_STRONG_INLINE void setScaleFactor(scalar_t scaleFactor) {
            mScaleFactor = scaleFactor;
        }

        EIGEN_STRONG_INLINE scalar_t scaleFactor() {
            return mScaleFactor;
        }

    private:
        float mX, mY;
        float mAngle = 0;
        float mResponse = 0;
        float mScaleFactor = SCALEFACTOR;
        short mLevel = -1;
        short mSize = 0;
    };

    class OptimizationPair {

    public:
        explicit OptimizationPair(PFrame aFrame, PPoint aMapPoint, MapPointType aType) :
        frame(aFrame), mapPoint(aMapPoint), type(aType)
        { }

        explicit OptimizationPair(PFrame aFrame, PPoint aMapPoint, MapPointType aType, const Corner &aCorner) :
        frame(aFrame), mapPoint(aMapPoint), type(aType), corner(aCorner)
        { }

        PFrame frame;
        PPoint mapPoint;
        MapPointType type;
        Optional<Corner> corner;

    };

    inline List<NormalizedVector2d> normalize(const List<DistortedVector2d> &points, Matrix33 &normalizationMatrix) {

        scalar_t meanX = 0;
        scalar_t meanY = 0;
        const int N = points.size();

        List<NormalizedVector2d> result;
        result.resize(N);

        for (int i = 0; i < N; i++)
        {
            meanX += points[i].x();
            meanY += points[i].y();
        }

        meanX = meanX / (scalar_t)N;
        meanY = meanY / (scalar_t)N;

        scalar_t meanDevX = 0;
        scalar_t meanDevY = 0;

        for(int i = 0; i < N; i++)
        {
            result[i].x() = points[i].x() - meanX;
            result[i].y() = points[i].y() - meanY;

            meanDevX += fabs(result[i].x());
            meanDevY += fabs(result[i].y());
        }

        meanDevX = meanDevX/N;
        meanDevY = meanDevY/N;

        scalar_t sX = 1.0 / meanDevX;
        scalar_t sY = 1.0 / meanDevY;

        for(int i=0; i<N; i++)
        {
            result[i].x() = result[i].x() * sX;
            result[i].y() = result[i].y() * sY;
        }

        normalizationMatrix.setIdentity();
        normalizationMatrix(0,0) = sX;
        normalizationMatrix(1,1) = sY;
        normalizationMatrix(0,2) = -meanX*sX;
        normalizationMatrix(1,2) = -meanY*sY;

        return result;

    }

    typedef struct FeatureIndex {

        FeatureIndex(short group, short index) {
            this->group = group;
            this->index = index;
        }

        FeatureIndex() {
            group = -1;
            index = -1;
        }

        bool hasValidValue() {
            return group >= 0 && index >= 0;
        }

        short group;
        short index;

        size_t operator()(const FeatureIndex &i) const {
            return group * 4096 + index;
        }

        bool operator==(const FeatureIndex &other) const {
            return this->group == other.group && this->index == other.index;
        }

    } FeatureIndex;

    typedef struct NearestNeighbor {
        NearestNeighbor() {
        }
        NearestNeighbor(size_t _index, scalar_t _distance) : index(_index), distance(_distance) {
        }
        size_t index;
        scalar_t distance;
    } NearestNeighbor;

    template <typename T> class PointGrid;

    using Pattern = std::vector<Vector2>;

    namespace PredefinedPattern {

        EIGEN_STRONG_INLINE Pattern single() {
            const Pattern single = {
                    {0, 0}
            };
            return single;
        }

        EIGEN_STRONG_INLINE Vector2 single(size_t i) {
            const Pattern single = {
                    {0, 0}
            };
            return single[i];
        }

        EIGEN_STRONG_INLINE Pattern star8() {
            const Pattern star8 = {
                    {0,-2},
                    {-1,-1},
                    {1,-1},
                    {-2,0},
                    {0,0},
                    {2,0},
                    {-1,1},
                    {0,2}
            };
            return star8;
        }

        EIGEN_STRONG_INLINE Vector2 star8(size_t i) {
            const Pattern star8 = {
                    {0,-2},
                    {-1,-1},
                    {1,-1},
                    {-2,0},
                    {0,0},
                    {2,0},
                    {-1,1},
                    {0,2}
            };
            return star8[i];
        }

        EIGEN_STRONG_INLINE Pattern star9() {
            const Pattern star9 = {
                    {0, 0},
                    {0, 2},
                    {2, 0},
                    {0, -2},
                    {-2, 0},
                    {1, 1},
                    {-1, 1},
                    {1, -1},
                    {-1, -1}
            };
            return star9;
        }

        EIGEN_STRONG_INLINE Vector2 star9(size_t i) {
            const Pattern star9 = {
                    {0, 0},
                    {0, 2},
                    {2, 0},
                    {0, -2},
                    {-2, 0},
                    {1, 1},
                    {-1, 1},
                    {1, -1},
                    {-1, -1}
            };
            return star9[i];
        }

        EIGEN_STRONG_INLINE Pattern fullSpread21() {
            const Pattern fullSpread21 = {
                    {0,-2},
                    {-1,-1},
                    {1,-1},
                    {-2,0},
                    {0,0},
                    {2,0},
                    {-1,1},
                    {1,1},
                    {0,2},
                    {-2,-2},
                    {-2,2},
                    {2,-2},
                    {2,2},
                    {-3,-1},
                    {-3,1},
                    {3,-1},
                    {3,1},
                    {1,-3},
                    {-1,-3},
                    {1,3}
            };
            return fullSpread21;
        }

        EIGEN_STRONG_INLINE Vector2 fullSpread21(size_t i) {
            const Pattern fullSpread21 = {
                    {0,-2},
                    {-1,-1},
                    {1,-1},
                    {-2,0},
                    {0,0},
                    {2,0},
                    {-1,1},
                    {1,1},
                    {0,2},
                    {-2,-2},
                    {-2,2},
                    {2,-2},
                    {2,2},
                    {-3,-1},
                    {-3,1},
                    {3,-1},
                    {3,1},
                    {1,-3},
                    {-1,-3},
                    {1,3}
            };
            return fullSpread21[i];
        }

    };
}

#endif
