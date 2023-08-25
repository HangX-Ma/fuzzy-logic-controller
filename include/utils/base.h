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

#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED              "\x1b[1;31m"
#endif

#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN            "\x1b[21;32m"
#endif

#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW           "\x1b[21;33m"
#endif

#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE             "\x1b[21;34m"
#endif

#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA          "\x1b[21;35m"
#endif

#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN             "\x1b[21;36m"
#endif

#ifndef ANSI_COLOR_WHITE
#define ANSI_COLOR_WHITE            "\x1b[21;37m"
#endif

#ifndef ANSI_COLOR_BRIGHT_BLACK
#define ANSI_COLOR_BRIGHT_BLACK     "\x1b[21;90m"
#endif

#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET            "\x1b[0m"
#endif
namespace fc {


#ifdef FC_USE_DEBUG_MSG
#define dbgmsg(fmt, ...) printf(ANSI_COLOR_BLUE "[DEBUG]" fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define dbgmsgln(fmt, ...) printf(ANSI_COLOR_BLUE "[DEBUG]" fmt ANSI_COLOR_RESET"\n", ##__VA_ARGS__)
#else
#define dbgmsg(fmt, ...)
#define dbgmsgln(fmt, ...)
#endif

#ifdef FC_USE_INFO_MSG
#define infomsg(fmt, ...) printf(ANSI_COLOR_YELLOW "[INFO]" fmt ANSI_COLOR_RESET, ##__VA_ARGS__)
#define infomsgln(fmt, ...) printf(ANSI_COLOR_YELLOW "[INFO]" fmt ANSI_COLOR_RESET"\n", ##__VA_ARGS__)
#else
#define infomsg(fmt, ...)
#define infomsgln(fmt, ...)
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

#endif  //!__FC_BASE__H__