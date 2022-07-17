/**
 * @file Operation.h
 * @author mahx(MContour) m-contour@qq.com
 * @brief Operation class for simple calculation usage
 * @version 0.1
 * @date 2022-07-14
 * 
 * @copyright Copyright (c) 2022 Fuzzy Limited. All rights reserved.
 * 
 */

#ifndef __OPERATION__H__
#define __OPERATION__H__

#include "Base.h"

#include <vector>


namespace fc
{
    class Operation {
        public:
            template <typename T>
            static T min(T a, T b);

            template <typename T>
            static T max(T a, T b);

            template <typename T>
            static bool isInf(T x);

            template <typename T>
            static bool isNaN(T x);

            template <typename T>
            static bool isFinite(T x);

            static bool isEqual(scalar a, scalar b, scalar threshold = eps);

            static bool isGreaterThan(scalar a, scalar b, scalar threshold = eps);

            static bool isGreaterOrEqual(scalar a, scalar b, scalar threshold = eps);

            static bool isLessthan(scalar a, scalar b, scalar threshold = eps);

            static bool isLessOrEqual(scalar a, scalar b, scalar threshold = eps);

            static scalar toScalar(const std::string& x);
    };
}


namespace fc {
    template <typename T>
    inline T Operation::min(T a, T b) {
        if (Operation::isNaN(a)) return b;
        if (Operation::isNaN(b)) return a;

        return a < b ? a : b;
    }

    template <typename T>
    inline T Operation::max(T a, T b) {
        if (Operation::isNaN(a)) return b;
        if (Operation::isNaN(b)) return a;

        return a > b ? a : b;
    }

    template <typename T>
    inline bool Operation::isInf(T x) {
        return x == fc::inf || x == -fc::inf;
    }


    template <typename T>
    inline bool Operation::isNaN(T x) {
        return (x != x);
    } // NaN values never compare equal to themselves or to other NaN values. Copying a NaN is not required.


    template<typename T>
    inline bool Operation::isFinite(T x) {
        return not (x != x || x == fc::inf || x == -fc::inf);
    }

    inline bool Operation::isEqual(scalar a, scalar b, scalar threshold) {
        return a == b || std::abs(a - b) < threshold || (a != a && b != b);
    }


    inline bool Operation::isGreaterThan(scalar a, scalar b, scalar threshold) {
        return !(a == b || std::abs(a - b) < threshold || (a != a && b != b)) && a > b;
    }


    inline bool Operation::isGreaterOrEqual(scalar a, scalar b, scalar threshold) {
        return a == b || std::abs(a - b) < threshold || (a != a && b != b) || a > b;
    }


    inline bool Operation::isLessthan(scalar a, scalar b, scalar threshold) {
        return !(a == b || std::abs(a - b) < threshold || (a != a && b != b)) && a < b;
    }


    inline bool Operation::isLessOrEqual(scalar a, scalar b, scalar threshold) {
        return (a == b || std::abs(a - b) < threshold || (a != a && b != b)) || a < b;
    }

    inline scalar Operation::toScalar(const std::string& x) {
        std::istringstream iss(x);
        scalar result;
        iss >> result;
        char strict;
        if (!(iss.fail() || iss.get(strict))) {
            return result;
        }

        std::ostringstream _nan;
        _nan << fc::nan;
        if (x == _nan.str() || x == "nan")
            return fc::nan;

        std::ostringstream pInf;
        pInf << fc::inf;
        if (x == pInf.str() || x == "inf")
            return fc::inf;

        std::ostringstream nInf;
        nInf << (-fc::inf);
        if (x == nInf.str() || x == "-inf")
            return -fc::inf;
    }

}






#endif  //!__OPERATION__H__