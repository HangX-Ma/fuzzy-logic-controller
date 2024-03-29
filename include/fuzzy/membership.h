/**
 * @file membership.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief membership functions class
 * @version 0.3
 * @date 2022-07-15
 * @date 2023-08-22
 * @date 2024-01-16
 */

#ifndef FC_MEMBERSHIP_H
#define FC_MEMBERSHIP_H

#include "fuzzy/utils.h"
#include <optional>

namespace fc
{

enum class MembershipType : uint8_t
{
    None = 0,
    Gaussian = 2,
    Triangle = 3,
    Trapezoid = 4,
};

using Range = std::pair<scalar, scalar>;

class Membership
{
public:
    explicit Membership(std::string name = "", scalar height = 1.0);
    ~Membership() = default;

    std::optional<scalar> calculate(scalar input, const std::vector<scalar> &param_set);

    void setMembershipParam(MembershipType type, const scalar input_params[], uint8_t params_num);
    void setHeight(scalar height);

    scalar getHeight() const;
    size_t getDiscourseSize() const;
    std::vector<scalar> getParamSet(size_t discourse_id);
    Range getRange(size_t discourse_id) const;
    MembershipType getType() const;
    const std::string &getName() const;
    scalar getMinimum() const;
    scalar getMaximum() const;

private:
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
    scalar triangle(scalar x = fc::nan, scalar vertexA = fc::nan, scalar vertexB = fc::nan,
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
    scalar trapezoid(scalar x = fc::nan, scalar vertexA = fc::nan, scalar vertexB = fc::nan,
                     scalar vertexC = fc::nan, scalar vertexD = fc::nan) const;

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
    scalar gaussian(scalar x = fc::nan, scalar mean = fc::nan,
                    scalar standardDeviation = fc::nan) const;

    Range calculateRange(MembershipType type, const std::vector<scalar> &param_set);

private:
    scalar height_;
    std::string name_;
    MembershipType type_;
    size_t discourse_size_;
    std::vector<std::vector<scalar>> params_;
    std::vector<Range> params_range_;
    scalar minimum_;
    scalar maximum_;
};

} // namespace fc

#endif