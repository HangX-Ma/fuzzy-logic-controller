/**
 * @file pid.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzy logic
 * @version 0.1
 * @date 2023-08-28
 */

#ifndef __FC_FUZZY_PID__H__
#define __FC_FUZZY_PID__H__

#include "utils/base.h"
#include "utils/operation.h"

namespace fc {

class PController {
    public:
        PController();
        ~PController();

        scalar algo(const scalar err);

        void setProportional(const scalar Kp);
        scalar getProportional(void);
    private:
        scalar Kp_;
};

typedef struct PID {
    scalar Kp;
    scalar Ki;
    scalar Kd;
} PID_t;

class PIDController {
    public:
        PIDController();
        ~PIDController();

        scalar algo(const scalar err);

        void setPIDParams(const PID_t pid);
        void clearIntegralPrev(void);

        scalar prev_err_;
    private:
        PID_t pid_;
        scalar integral_prev_;
        scalar integral_saturation_;
};

}

#endif  //!__FC_FUZZY_PID__H__