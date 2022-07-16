/**
 * @file Base.h
 * @author mahx(MContour) m-contour@qq.com
 * @brief Basic definitions
 * @version 0.1
 * @date 2022-07-14
 * 
 * @copyright Copyright (c) 2022 Fuzzy Limited. All rights reserved.
 * 
 */

#ifndef __BASE__H__
#define __BASE__H__

#include <cmath>
#include <limits>
#include <iostream>
#include <ostream>
#include <sstream>
#include <algorithm>

namespace fc {

/* basic type definition */
#ifdef  FC_USE_FLOAT
    typedef float scalar;
#else
    typedef double scalar;
#endif


#ifdef __GNUC__
#define FC_UNUSED_VAL __attribute__((unused))
#else
#define FC_UNUSED_VAL
#endif

    /* represent the Not-A-Number and inifinity value */
    const scalar nan FC_UNUSED_VAL = std::numeric_limits<scalar>::quiet_NaN();
    const scalar inf FC_UNUSED_VAL = std::numeric_limits<scalar>::infinity();
    const scalar eps = 1e-6;

    /* alias for `null` */
    const std::nullptr_t null = nullptr;

}
#endif  //!__BASE__H__