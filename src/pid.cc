#include "fuzzy/pid.h"

using namespace fc;

PController::PController() : Kp_(0) {}
PController::~PController() {}

scalar PController::algo(const scalar err) {
    return Kp_ * err;
}

void PController::setProportional(const scalar Kp) {
    if (Kp < -fc::eps) {
        warnmsgln("[PController] input Kp is negative and it will be set to zero");
        Kp_ = 0.0;
    }
    Kp_ = Kp;
}

PIDController::PIDController()
    : prev_err_(0),
      pid_(PID_t{0, 0, 0}),
      integral_prev_(0),
      integral_saturation_(0) {}

PIDController::~PIDController() {}

scalar PIDController::algo(const scalar err) {
    scalar proportional, integral, derivative , output;

    // u_p  = P *e(k)
    proportional = pid_.Kp * err;

    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    integral = integral_prev_ + pid_.Ki * (err + prev_err_) * 0.5;

    if (integral > fabs(integral_saturation_)) {
        integral = fabs(integral_saturation_);
    }
    if (integral < -fabs(integral_saturation_)) {
        integral = -fabs(integral_saturation_);
    }

    // u_dk = D(ek - ek_1)/Ts
    derivative = pid_.Kd * (err - prev_err_);

    output = proportional + integral + derivative;

    // save data for next control period
    integral_prev_ = integral;
    prev_err_      = err;

    return output;
}

void PIDController::setPIDParams(const PID_t pid) {
    if (pid.Kp < -fc::eps) {
        warnmsgln("[PIDController] input Kp is negative and it will be set to zero");
        pid_.Kp = 0.0;
    }

    if (pid.Ki < -fc::eps) {
        warnmsgln("[PIDController] input Ki is negative and it will be set to zero");
        pid_.Ki = 0.0;
    }

    if (pid.Kd < -fc::eps) {
        warnmsgln("[PIDController] input Kd is negative and it will be set to zero");
        pid_.Kd = 0.0;
    }
}

void PIDController::clearIntegralPrev(void) {
    integral_prev_ = 0.0;
}