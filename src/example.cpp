#include "FuzzyControl.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;


int main (int argc,char *argv[]) {
    fc::scalar goal = 500;
    fc::scalar curr = 0;
    fc::scalar u    = 0;

//    constexpr fc::scalar eNB  = -630, eNM  = -420, eNS  = -210, eZO  = 0, ePS  = 210, ePM  = 420, ePB  = 630;
//    constexpr fc::scalar deNB = -75, deNM = -50, deNS = -25, deZO = 0, dePS = 25, dePM = 50, dePB = 75;
//    constexpr fc::scalar uNB  = -180, uNM  = -120, uNS  = -60,  uZO  = 0, uPS  = 60,  uPM  = 120, uPB  = 180;

   constexpr fc::scalar eNB  = -330, eNM  = -220, eNS  = -110, eZO  = 0,   ePS  = 110, ePM  = 220, ePB  = 330;
   constexpr fc::scalar deNB = -240, deNM = -160, deNS = -80,  deZO = 0,   dePS = 80,  dePM = 160, dePB = 240;
   constexpr fc::scalar uNE  = -90, uNB  = -50,  uNM  = -20,  uNS  = -10, uZO  = 0,   uPS  = 10,  uPM  = 20, uPB = 50, uPE = 90;

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
    std::vector<fc::scalar> u_mf_paras {uNE,uNB,uNM, uNB,uNM,uNS, uNM,uNS,uZO, uNS,uZO,uPS, uZO,uPS,uPM, uPS,uPM,uPB, uPM,uPB,uPE};

    // coff*Kp_u*sign(i)*i^2
    // std::vector<fc::scalar> e_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    // std::vector<fc::scalar> de_mf_paras {-3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3};
    // std::vector<fc::scalar> u_mf_paras {-9,-9,-4,-9,-4,-1,-4,-1,0,-1,0,1,0,1,4,1,4,9,4,9,9};

    fc::FuzzyController* fzc = new fc::FuzzyController(330,240,90);

    fzc->setMembershipType_err(fc::membershipType::Triangle);
    fzc->setMembershipType_err_dev(fc::membershipType::Triangle);
    fzc->setMembershipType_u(fc::membershipType::Triangle);

    fzc->set_err_param(e_mf_paras);
    fzc->set_err_dev_param(de_mf_paras);
    fzc->set_u_param(u_mf_paras);

    fzc->setFuzzyRule(ruleMatrix);
    fzc->setResolution(100);
    fzc->setParam_K(0.65, 0.42, 2.5, 0.0006, 0.02);
    fzc->showInfo();

    uint8_t iter_max = 250; 
    std::vector<fc::scalar> epoch(iter_max), err(iter_max), err_dev(iter_max), base(iter_max,0);

    epoch.at(0)         = 0;
    err.at(0)           = 0;
    err_dev.at(0)       = 0;

    for(int i = 1; i < iter_max; i++) {
        if (i >= 50) goal = 160;
        if (i >= 80) goal = 480;
        if (i >= 120) goal = 0;
        if (i >= 160) goal = 960;

        epoch.at(i) = i;
        err.at(i) = goal - curr;
        err_dev.at(i) = err.at(i) - err.at(i-1);
        printf("err=%f, err_dev=%f\n", err.at(i), err_dev.at(i));

        u = fzc->algo(goal,curr);
        curr += u*0.01*150;
    }

    // Set the size of output image to 1200x780 pixels
    plt::figure_size(1200, 780);
    // plot base 
    plt::plot(epoch, base, "r--");
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