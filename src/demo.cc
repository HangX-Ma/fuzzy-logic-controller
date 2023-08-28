#define _USE_MATH_DEFINES
#include "fuzzy/fuzzy_def.h"
#include "fuzzy/fuzzy_logic.h"
#include <iostream>
#include <cmath>
#include <vector>

constexpr const size_t TRIANGLE_PARAMS_NUM = 21;
constexpr const size_t TRAPEZOID_PARAMS_NUM = 28;
constexpr const size_t GAUSSIAN_PARAMS_NUM = 14;

int main (int argc,char *argv[]) {
    FC_UNUSED(argc);
    FC_UNUSED(argv);

    // membership triangle params
    fc::scalar triangle_params[TRIANGLE_PARAMS_NUM] = {
        -3, -3, -2,
        -3, -2, -1,
        -2, -1,  0,
        -1,  0,  1,
         0,  1,  2,
         1,  2,  3,
         2,  3,  3,
    };
    // membership trapezoid params
    fc::scalar trapezoid_params[TRAPEZOID_PARAMS_NUM] = {
        -3.0, -3.0, -2.5, -2.0,
        -3.0, -2.5, -1.5, -1.0,
        -2.0, -1.5, -0.5,  0.0,
        -1.0, -0.5,  0.5,  1.0,
         0.0,  0.5,  1.5,  2.0,
         1.0,  1.5,  2.5,  3.0,
         2.0,  2.5,  3.0,  3.0,
    };
    // membership gaussian params
    fc::scalar gaussian_params[GAUSSIAN_PARAMS_NUM] = {
        -3.0, 0.5,
        -2.0, 0.4,
        -1.0, 0.3,
         0.0, 0.2,
         1.0, 0.3,
         2.0, 0.4,
         3.0, 0.5,
    };
    FC_UNUSED(trapezoid_params);
    FC_UNUSED(gaussian_params);

    fc::FuzzyLogic fuzzy;
    fuzzy.e->init(100, false, fc::membershipType::Triangle, triangle_params, TRIANGLE_PARAMS_NUM);
    // fuzzy.ec->init(200, false, fc::membershipType::Trapezoid, trapezoid_params, TRAPEZOID_PARAMS_NUM);
    // fuzzy.u->init(30, true, fc::membershipType::Gaussian, gaussian_params, GAUSSIAN_PARAMS_NUM);
    fuzzy.ec->init(50, false, fc::membershipType::Triangle, triangle_params, TRIANGLE_PARAMS_NUM);
    fuzzy.u->init(50, true, fc::membershipType::Triangle, triangle_params, TRIANGLE_PARAMS_NUM);

    size_t rows = fuzzy.e->membership_->getDiscourseSize();
    size_t cols = fuzzy.ec->membership_->getDiscourseSize();

    fc::Matrix rule_table;
    rule_table.resize(rows, cols);
    rule_table << PB,PB,PM,PM,PS,ZO,ZO,
                  PB,PM,PM,PS,PS,ZO,NS,
                  PM,PM,PS,PS,ZO,NS,NS,
                  PM,PM,PS,ZO,NS,NM,NM,
                  PS,PS,ZO,NS,NS,NM,NM,
                  PS,ZO,NS,NM,NM,NB,NB,
                  ZO,ZO,NM,NM,NM,NB,NB;

    fuzzy.setFuzzyRules(rule_table);

    fuzzy.plotFuzzyControlSurface(true);
    fuzzy.e->plotMembershipFunctions();
    fuzzy.ec->plotMembershipFunctions();
    fuzzy.u->plotMembershipFunctions();

    fuzzy.p_ctrl_->setProportional(4.0);

    // DON'T CHANGE FACTOR RATIO OUT OF RANGE!
    fuzzy.u->setFactor(2.0, true);

    fuzzy.getInfo();

#define CONTROL_TEST_CONSTANT_TARGET    (0)
#define CONTROL_TEST_SINE_TARGET        (1)
#if CONTROL_TEST_CONSTANT_TARGET
    fc::Control_t control;
    control.target = 60.0;
    control.actual = 0.0;

    const int times = 300;
    for (int i = 0; i < times; i++) {
        if (i == 50) {
            control.target = 100.0;
        } else if (i == 100) {
            control.target = -50.0;
        } else if (i == 200) {
            control.target = 40.0;
        }
        control.actual += fuzzy.algo(control, true, 1.6);
    }

    fuzzy.plotControl("_constant");
    fuzzy.plotControlErr("_constant");
#elif CONTROL_TEST_SINE_TARGET
    fc::Control_t control;
    control.target = 0.0;
    control.actual = 0.0;

    const int times = 5 * 360;
    for (int i = 0; i < times; i++) {
        if (i <= 360) {
            control.target = 2 * sin(i * M_PI / 360);
        } else if (i <= 2 * 360) {
            control.target = 5 * sin(i * M_PI / 180);
        } else if (i <= 3 * 360) {
            control.target = 10 * sin(i * M_PI / 90);
        } else if (i <= 4 * 360) {
            control.target = 15 * sin(i * M_PI / 30);
        } else if (i <= 5 * 360) {
            control.target = 25 * sin(i * M_PI / 720);
        }
        control.actual += fuzzy.algo(control, true, 1.6);
    }

    fuzzy.plotControl("_sine");
    fuzzy.plotControlErr("_sine");
#endif

    return 0;
}