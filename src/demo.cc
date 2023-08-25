#include "fuzzy/fuzzy_def.h"
#include "fuzzy/fuzzy_logic.h"
#include <iostream>
#include <cmath>
#include <vector>

constexpr const size_t E_PARAMS_NUM = 21;
constexpr const size_t EC_PARAMS_NUM = 28;
constexpr const size_t U_PARAMS_NUM = 14;

int main (int argc,char *argv[]) {
    FC_UNUSED(argc);
    FC_UNUSED(argv);


    // membership triangle params
    fc::scalar e_params[E_PARAMS_NUM] = {
        -3, -3, -2,
        -3, -2, -1,
        -2, -1,  0,
        -1,  0,  1,
         0,  1,  2,
         1,  2,  3,
         2,  3,  3,
    };
    // membership trapezoid params
    fc::scalar ec_params[EC_PARAMS_NUM] = {
        -3.0, -3.0, -2.5, -2.0,
        -3.0, -2.5, -1.5, -1.0,
        -2.0, -1.5, -0.5,  0.0,
        -1.0, -0.5,  0.5,  1.0,
         0.0,  0.5,  1.5,  2.0,
         1.0,  1.5,  2.5,  3.0,
         2.0,  2.5,  3.0,  3.0,
    };
    // membership gaussian params
    fc::scalar u_params[U_PARAMS_NUM] = {
        -3.0, 0.5,
        -2.0, 0.4,
        -1.0, 0.3,
         0.0, 0.2,
         1.0, 0.3,
         2.0, 0.4,
         3.0, 0.5,
    };

    fc::FuzzyLogic fuzzy;
    fuzzy.e->init(400, false, fc::membershipType::Triangle, e_params, E_PARAMS_NUM);
    fuzzy.ec->init(200, false, fc::membershipType::Trapezoid, ec_params, EC_PARAMS_NUM);
    fuzzy.u->init(30, true, fc::membershipType::Gaussian, u_params, U_PARAMS_NUM);

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

    fuzzy.plotFuzzyControlSurface();
    fuzzy.e->plotMembershipFunctions();
    fuzzy.ec->plotMembershipFunctions();
    fuzzy.u->plotMembershipFunctions();

    return 0;
}