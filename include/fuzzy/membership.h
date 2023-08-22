/**
 * @file membership.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief membership functions class
 * @version 0.1
 * @date 2022-07-15
 * @date 2023-08-22
 */

#ifndef __FC_MEMBERSHIP__H__
#define __FC_MEMBERSHIP__H__

#include "base.h"
#include "operation.h"

namespace fc {
class Membership {
    public:
        explicit Membership(scalar height = 1.0);
        ~Membership();

        /**
         * @brief Compute membership function value at \f$x\f$
         * @note @f$\begin{cases}
         * 1h & \mbox{if $x \in [s, e]$} \cr
         * 0h & \mbox{otherwise}
         * \end{cases}@f$
         *
         * where \c h is the height of the Term,
         *       \c s is the start of the Rectangle,
         *       \c e is the end of the Rectangle.
         *
         * @param [in] x computation position
         * @param [in] start
         * @param [in] end
         * @return membership value \f$\mu(x)\f$
         */
        scalar Rectangle(scalar x     = fc::nan,
                         scalar start = fc::nan,
                         scalar end   = fc::nan) const;

        /**
         * @brief Compute membership function value at \f$x\f$
         * @note @f$\begin{cases}
         * 0h & \mbox{if $x \not\in [a,c]$}\cr
         * 1h & \mbox{if $x = b$}\cr
         * h (x - a) / (b - a) & \mbox{if $x < b$} \cr
         * h (c - x) / (c - b) & \mbox{otherwise}
         * \end{cases} @f$
         *
         * where \c h is the height of the Term,
         *       \c a is the first vertex of the Triangle,
         *       \c b is the second vertex of the Triangle,
         *       \c c is the third vertex of the Triangle
         * @param [in] x computation position
         * @param [in] vertexA
         * @param [in] vertexB
         * @param [in] vertexC
         * @return membership value \f$\mu(x)\f$
         */
        scalar Triangle(scalar x       = fc::nan,
                        scalar vertexA = fc::nan,
                        scalar vertexB = fc::nan,
                        scalar vertexC = fc::nan) const;

        /**
         * @brief Compute membership function value at \f$x\f$
         * @note @f$\begin{cases}
         * 0h & \mbox{if $x \not\in[a,d]$}\cr
         * h \times \min(1, (x - a) / (b - a)) & \mbox{if $x < b$}\cr
         * 1h & \mbox{if $x \leq c$}\cr
         *  h (d - x) / (d - c) & \mbox{if $x < d$}\cr
         *  0h & \mbox{otherwise}
         *  \end{cases}@f$
         *
         * where \c h is the height of the Term,
         *       \c a is the first vertex of the Trapezoid,
         *       \c b is the second vertex of the Trapezoid,
         *       \c c is the third vertex of the Trapezoid
         *       \c d is the forth vertex of the Trapezoid
         *
         * @param [in] x computation position
         * @param [in] vertexA
         * @param [in] vertexB
         * @param [in] vertexC
         * @param [in] vertexD
         * @return membership value \f$\mu(x)\f$
         */
        scalar Trapezoid(scalar x       = fc::nan,
                         scalar vertexA = fc::nan,
                         scalar vertexB = fc::nan,
                         scalar vertexC = fc::nan,
                         scalar vertexD = fc::nan) const;

        /**
         * @brief Compute membership function value at \f$x\f$
         * @note @f$ h \times \exp(-(x-\mu)^2/(2\sigma^2))@f$
         *
         * where @f$h@f$ is the height of the Term,
         *       @f$\mu@f$ is the mean of the Gaussian,
         *       @f$\sigma@f$ is the standard deviation of the Gaussian
         * @param [in] x computation position
         * @param [in] mean
         * @param [in] standardDeviation
         * @return membership value \f$\mu(x)\f$
         */
        scalar Gaussian(scalar x                 = fc::nan,
                        scalar mean              = fc::nan,
                        scalar standardDeviation = fc::nan) const;

        void setHeight(scalar height);

        scalar getHeight() const;

    private:
        scalar m_height;

};
}


#endif  //!__FC_MEMBERSHIP__H__