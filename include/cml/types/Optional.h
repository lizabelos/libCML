#ifndef CML_OPTIONAL_H
#define CML_OPTIONAL_H

namespace CML {

    template <typename T> class Optional {

    public:
        inline Optional() {
            mHasValue = false;
        }

        inline Optional(const T &value) : mValue(value) {
            mHasValue = true;
        }

        inline bool has_value() const {
            return mHasValue;
        }

        inline const T &value() const {
            return mValue;
        }

        inline T &value() {
            return mValue;
        }

        inline const T &valueOrDefault(const T &v) const {
            if (mHasValue) {
                return mValue;
            } else {
                return v;
            }
        }

        inline const T &valueOrAbort() const {
            if (mHasValue) {
                return mValue;
            } else {
                abort();
            }
        }

    private:
        bool mHasValue;
        T mValue;

    };

}

#endif //CML_OPTIONAL_H
