/**
 * @file pid.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzy logic
 * @version 0.2
 * @date 2023-08-28
 * @date 2024-01-16
 */

#ifndef FC_PID_H
#define FC_PID_H

#include "fuzzy/utils.h"

namespace fc
{

class PController
{
public:
    scalar algo(scalar err);

    void setProportional(scalar Kp);
    scalar getProportional();

private:
    scalar Kp_{0.0};
};

using PID_t = struct PID
{
    scalar Kp;
    scalar Ki;
    scalar Kd;
};

class PIDController
{
public:
    PIDController();
    ~PIDController();

    scalar algo(scalar err);

    void setPIDParams(PID_t pid);
    void clearSaturation() { integral_prev_ = 0.0; }
    scalar getPrevErr() const { return prev_err_; }

private:
    PID_t pid_{0.0, 0.0, 0.0};
    scalar prev_err_{0};
    scalar integral_prev_{0.0};
    scalar integral_saturation_{0.0};
};

} // namespace fc

#endif