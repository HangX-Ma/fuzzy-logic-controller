#include "FuzzyControl.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;


int main (int argc,char *argv[]) {
    fc::scalar goal = 60;
    fc::scalar curr = 0;
    fc::scalar u    =0;

    int8_t ruleMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
                            {NB,NB,NM,NS,NS,ZO,PS},
                            {NM,NM,NM,NS,ZO,PS,PS},
                            {NM,NM,NS,ZO,PS,PM,PM},
                            {NS,NS,ZO,PS,PS,PM,PM},
                            {NS,ZO,PS,PM,PM,PM,PB},
                            {ZO,ZO,PM,PM,PM,PB,PB}};

    // input parameter number is [N x M]
    std::vector<fc::scalar> e_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    std::vector<fc::scalar> de_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    std::vector<fc::scalar> u_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};

    fc::FuzzyController* fzc = new fc::FuzzyController(100,65,50);

    fzc->setMembershipType_err(fc::membershipType::Triangle);
    fzc->setMembershipType_err_dev(fc::membershipType::Triangle);
    fzc->setMembershipType_u(fc::membershipType::Triangle);

    fzc->set_err_param(e_mf_paras);
    fzc->set_err_dev_param(de_mf_paras);
    fzc->set_u_param(u_mf_paras);

    fzc->setFuzzyRule(ruleMatrix);
    fzc->setResolution(100);
    fzc->showInfo();

    uint8_t iter_max = 50; 
    std::vector<fc::scalar> epoch(iter_max), err(iter_max), err_dev(iter_max);

    epoch.at(0)         = 0;
    err.at(0)           = goal - curr;
    err_dev.at(0)       = err.at(0);

    for(int i = 1; i < iter_max; i++) {
        epoch.at(i) = i;
        err.at(i) = goal - curr;
        err_dev.at(i) = err.at(i) - err.at(i-1);

        u = fzc->algo(goal,curr);
        curr+=u;
    }

    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // plot error
    plt::named_plot("error", epoch, err);
    // plot derivative error
    plt::named_plot("derivative error", epoch, err_dev);
    // Add graph title
    plt::title("Fuzzy Controller Example");
    // Enable legend.
    plt::legend();
    // Save the image (file format is determined by the extension)
    plt::save("../../share/example.png");

    delete fzc;

    return 1;
}