/**
 * @file base.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Basic definitions
 * @version 0.2
 * @date 2022-07-14
 * @date 2023-08-22
 */

#ifndef __FC_BASE__H__
#define __FC_BASE__H__

#include <limits>
#include "Eigen/Eigen"

namespace fc {


#ifdef FC_USE_DEBUG_MSG
#define dbgmsg(fmt, ...) printf("[DEBUG]" fmt, ##__VA_ARGS__)
#define dbgmsgln(fmt, ...) printf("[DEBUG]" fmt "\n", ##__VA_ARGS__)
#else
#define dbgmsg(fmt, ...)
#define dbgmsgln(fmt, ...)
#endif

/* basic type definition */
#ifdef FC_USE_FLOAT
    typedef float scalar;
    typedef Eigen::MatrixXf Matrix;
#elif FC_USE_DOUBLE
    typedef double scalar;
    typedef Eigen::MatrixXd Matrix;
#endif

#define FC_UNUSED(x) (void)(x)

#ifdef __GNUC__
#define FC_UNUSED_DECLARE __attribute__((unused))
#else
#define FC_UNUSED_DECLARE
#endif

/* represent the Not-A-Number and infinity value */
const scalar nan = std::numeric_limits<scalar>::quiet_NaN();
const scalar inf = std::numeric_limits<scalar>::infinity();
const scalar eps = 1e-6;

}

#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif

#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif

#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif

#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif

#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif

#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif

#ifndef ANSI_COLOR_WHITE
#define ANSI_COLOR_WHITE "\x1b[1;37m"
#endif

#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#endif  //!__FC_BASE__H__