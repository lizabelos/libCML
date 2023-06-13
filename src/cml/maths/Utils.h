//
// Created by tbelos on 19/07/19.
//

#ifndef CML_UTILS_H
#define CML_UTILS_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <random>
#include <chrono>
#include <queue>

namespace CML {

    template <typename T> T min(T a, T b) {
        if (a < b) return a;
        else return b;
    }

    template <typename T> T max(T a, T b) {
        if (a < b) return b;
        else return a;
    }

/*
    template <typename T> T mean(std::vector<T> v) {

        T sum = std::accumulate(v.begin(), v.end(), 0.0);
        T mean = sum / v.size();

        return mean;

    }
*/
    template <typename T> T mean(const List<T> &v) {

        T sum = std::accumulate(v.begin(), v.end(), T(0.0));
        T mean = sum / v.size();

        return mean;

    }
/*
    template <typename T> T stdev(std::vector<T> v) {

        T sum = std::accumulate(v.begin(), v.end(), T(0.0));
        T mean = sum / v.size();

        std::vector<T> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(), [mean](T x) { return x - mean; });
        T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0.0));
        T stdev = std::sqrt(sq_sum / v.size());

        return stdev;

    }
*/
    template <typename T> T stdev(const List<T> &v) {

        T sum = std::accumulate(v.begin(), v.end(), T(0.0));
        T mean = sum / v.size();

        std::vector<T> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(), [mean](T x) { return x - mean; });
        T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0.0));
        T stdev = std::sqrt(sq_sum / v.size());

        return stdev;

    }

    template <typename T> class MedianComputer {

    public:
        void initialize(const T &x) {
            s.push(x);
            med = x;
        }

        void addValue(const T &x) {
            if (!isInit) {
                initialize(x);
                isInit = true;
                return;
            }

            // case1(left side heap has more elements)
            if (s.size() > g.size())
            {
                if (x < med)
                {
                    g.push(s.top());
                    s.pop();
                    s.push(x);
                }
                else
                    g.push(x);

                med = (s.top() + g.top())/2.0;
            }

                // case2(both heaps are balanced)
            else if (s.size()==g.size())
            {
                if (x < med)
                {
                    s.push(x);
                    med = (T)s.top();
                }
                else
                {
                    g.push(x);
                    med = (T)g.top();
                }
            }

                // case3(right side heap has more elements)
            else
            {
                if (x > med)
                {
                    s.push(g.top());
                    g.pop();
                    g.push(x);
                }
                else
                    s.push(x);

                med = (s.top() + g.top())/2.0;
            }

        }

        T getMedian() {
            return med;
        }

        bool isInitialized() {
            return isInit;
        }

    private:
        bool isInit = false;
        std::priority_queue<T> s;
        std::priority_queue<T,std::vector<T>,std::greater<T>> g;
        T med;

    };

    // https://www.geeksforgeeks.org/median-of-stream-of-running-integers-using-stl/
    template <typename T> T median(const List<T> &list) {
        assertThrow(list.size() > 0, "Empty list for median");
        // max heap to store the smaller half elements
        std::priority_queue<T> s;

        // min heap to store the greater half elements
        std::priority_queue<T,std::vector<T>,std::greater<T>> g;

        T med = list[0];
        s.push(list[0]);

        // reading elements of stream one by one
        /*  At any time we try to make heaps balanced and
            their sizes differ by at-most 1. If heaps are
            balanced,then we declare median as average of
            min_heap_right.top() and max_heap_left.top()
            If heaps are unbalanced,then median is defined
            as the top element of heap of larger size  */
        for (size_t i = 1; i < list.size(); i++)
        {
            T x = list[i];

            // case1(left side heap has more elements)
            if (s.size() > g.size())
            {
                if (x < med)
                {
                    g.push(s.top());
                    s.pop();
                    s.push(x);
                }
                else
                    g.push(x);

                med = (s.top() + g.top())/2.0;
            }

                // case2(both heaps are balanced)
            else if (s.size()==g.size())
            {
                if (x < med)
                {
                    s.push(x);
                    med = (T)s.top();
                }
                else
                {
                    g.push(x);
                    med = (T)g.top();
                }
            }

                // case3(right side heap has more elements)
            else
            {
                if (x > med)
                {
                    s.push(g.top());
                    g.pop();
                    g.push(x);
                }
                else
                    s.push(x);

                med = (s.top() + g.top())/2.0;
            }

        }

        return med;

    }
/*
    template <typename T> T median(std::vector<T> v) {
        assertThrow(v.size() > 0, "Empty list for median");
        return median(List<T>(v.begin(), v.end()));

    }
*/
    template <typename T> T minimum(const std::vector<T> &v) {

        std::sort(v.begin(), v.end());
        return v[0];

    }

    template <typename T> T maximum(const std::vector<T> &v) {

        std::sort(v.begin(), v.end());
        return v[v.size() - 1];

    }
/*
    template <typename T> T mad(std::vector<T> v) {

        std::sort(v.begin(), v.end());

        T mean = v[v.size() / 2];

        std::vector<T> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(), [mean](T x) { return std::abs(x - mean); });

        std::sort(diff.begin(), diff.end());
        return diff[diff.size() / 2];

    }
*/
    template <typename T> T mad(const List<T> &v) {

        std::sort(v.begin(), v.end());

        T mean = v[v.size() / 2];

        std::vector<T> diff(v.size());
        std::transform(v.begin(), v.end(), diff.begin(), [mean](T x) { return std::abs(x - mean); });

        std::sort(diff.begin(), diff.end());
        return diff[diff.size() / 2];

    }

    template <typename T> T chi2(const T &value, const std::vector<T> &distribution) {

        T sum = T(0);
        for (T e : distribution) {
            sum += ((value - e) * (value - e)) / e;
        }
        return sum;

    }


}

#endif //CML_UTILS_H
