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
    fuzzy.u->init(30, true, fc::membershipType::Triangle, triangle_params, TRIANGLE_PARAMS_NUM);

    size_t rows = fuzzy.e->membership_->getDiscourseSize();
    size_t cols = fuzzy.ec->membership_->getDiscourseSize();

    fc::Matrix rule_table;
    rule_table.resize(rows, cols);
    rule_table << NB,NB,NM,NM,NS,ZO,ZO,
                  NB,NB,NM,NS,NS,ZO,PS,
                  NM,NM,NM,NS,ZO,PS,PS,
                  NM,NM,NS,ZO,PS,PM,PM,
                  NS,NS,ZO,PS,PS,PM,PM,
                  NS,ZO,PS,PM,PM,PM,PB,
                  ZO,ZO,PM,PM,PM,PB,PB;

    fuzzy.setFuzzyRules(rule_table);

    fuzzy.plotFuzzyControlSurface(true);
    fuzzy.e->plotMembershipFunctions();
    fuzzy.ec->plotMembershipFunctions();
    fuzzy.u->plotMembershipFunctions();

    fuzzy.getInfo();


    fc::Control_t control;
    control.target = 60.0;
    control.actual = 0.0;

    const int times = 10;
    for (int i = 0; i < times; i++) {
        control.actual += fuzzy.algo(control, true);
    }
    fuzzy.plotControl();
    fuzzy.plotControlErr();

    return 0;
}