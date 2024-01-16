/**
 * @file utils.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Basic definitions
 * @version 0.3
 * @date 2022-07-14
 * @date 2023-08-22
 * @date 2024-01-16
 */

#ifndef FC_UTILS_H
#define FC_UTILS_H

#include <limits>
#include <eigen3/Eigen/Eigen>

// clang-format off
namespace fc {

/* basic type definition */
#ifdef FC_USE_FLOAT
    typedef float scalar;
    typedef Eigen::MatrixXf Matrix;
#elif FC_USE_DOUBLE
    using scalar = double;
    using Matrix = Eigen::MatrixXd;
#endif

#define FC_UNUSED(x) (void)(x)

#ifdef __GNUC__
#define FC_UNUSED_DECLARE __attribute__((unused))
#else
#define FC_UNUSED_DECLARE
#endif

/* represent the Not-A-Number and infinity value */
constexpr scalar nan = std::numeric_limits<scalar>::quiet_NaN();
constexpr scalar inf = std::numeric_limits<scalar>::infinity();
constexpr scalar eps = 1e-7;


using PremisePair = std::pair<scalar, size_t>;
using CentroidPair = std::pair<scalar, scalar>;

}  // namespace fc

#endif