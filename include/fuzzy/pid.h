/**
 * @file pid.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzy logic
 * @version 0.1
 * @date 2023-08-28
 */

#ifndef __FC_FUZZY_PID__H__
#define __FC_FUZZY_PID__H__

#include "fuzzy/base.h"
#include "fuzzy/operation.h"

namespace fc {

class PController {
    public:
        PController();
        ~PController();

        scalar algo(scalar err);

        void setProportional(scalar Kp);
        scalar getProportional();
    private:
        scalar Kp_;
};

using PID_t = struct PID {
    scalar Kp;
    scalar Ki;
    scalar Kd;
};

class PIDController {
    public:
        PIDController();
        ~PIDController();

        scalar algo(scalar err);

        void setPIDParams(PID_t pid);
        void clearIntegralPrev();

        scalar prev_err_;
    private:
        PID_t pid_;
        scalar integral_prev_;
        scalar integral_saturation_;
};

}

#endif  //!__FC_FUZZY_PID__H__