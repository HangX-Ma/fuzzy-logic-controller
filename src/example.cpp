#include "FuzzyControl.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;


int main (int argc,char *argv[]) {
    fc::scalar goal = 480;
    fc::scalar curr = 0;
    fc::scalar u    = 0;

   constexpr fc::scalar eNB  = -640, eNM  = -400, eNS  = -210, eZO  = 0, ePS  = 210, ePM  = 400, ePB  = 640;
   constexpr fc::scalar deNB = -320, deNM = -200, deNS = -105, deZO = 0, dePS = 105, dePM = 200, dePB = 320;
   constexpr fc::scalar uNB  = -180, uNM  = -120, uNS  = -60,  uZO  = 0, uPS  = 60,  uPM  = 120, uPB  = 180; 

    int8_t ruleMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},
                            {NB,NB,NM,NS,NS,ZO,PS},
                            {NM,NM,NM,NS,ZO,PS,PS},
                            {NM,NM,NS,ZO,PS,PM,PM},
                            {NS,NS,ZO,PS,PS,PM,PM},
                            {NS,ZO,PS,PM,PM,PM,PB},
                            {ZO,ZO,PM,PM,PM,PB,PB}};

    // input parameter number is [N x M] coff*Kp_u*sign(i)*i
    std::vector<fc::scalar> e_mf_paras {eNB,eNB,eNM, eNB,eNM,eNS, eNM,eNS,eZO, eNS,eZO,ePS, eZO,ePS,ePM, ePS,ePM,ePB, ePM,ePB,ePB};
    std::vector<fc::scalar> de_mf_paras {deNB,deNB,deNM, deNB,deNM,deNS, deNM,deNS,deZO, deNS,deZO,dePS, deZO,dePS,dePM, dePS,dePM,dePB, dePM,dePB,dePB};
    std::vector<fc::scalar> u_mf_paras {uNB,uNB,uNM, uNB,uNM,uNS, uNM,uNS,uZO, uNS,uZO,uPS, uZO,uPS,uPM, uPS,uPM,uPB, uPM,uPB,uNB};

    // coff*Kp_u*sign(i)*i^2
    // std::vector<fc::scalar> e_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    // std::vector<fc::scalar> de_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    // std::vector<fc::scalar> u_mf_paras {-9,-9,-4,-9,-4,-1,-4,-1,0,-1,0,1,0,1,4,1,4,9,4,9,9};

    fc::FuzzyController* fzc = new fc::FuzzyController(640,320,180);

    fzc->setMembershipType_err(fc::membershipType::Triangle);
    fzc->setMembershipType_err_dev(fc::membershipType::Triangle);
    fzc->setMembershipType_u(fc::membershipType::Triangle);

    fzc->set_err_param(e_mf_paras);
    fzc->set_err_dev_param(de_mf_paras);
    fzc->set_u_param(u_mf_paras);

    fzc->setFuzzyRule(ruleMatrix);
    fzc->setResolution(100);
    fzc->setParam_K(0.5, 0.1, 3.0);
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
        curr += u*0.01*150;
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