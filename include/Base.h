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

//! Set red font in printf function
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
//! Set green font in printf function
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
//! Set yellow font in printf function
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
//! Set blue font in printf function
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
//! Set magenta font in printf function
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
//! Set cyan font in printf function
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
//! Set white font in printf function
#ifndef ANSI_COLOR_WHITE
#define ANSI_COLOR_WHITE "\x1b[1;37m"
#endif
//! Reset font color in printf function
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#endif  //!__BASE__H__