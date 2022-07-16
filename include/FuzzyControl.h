/**
 * @file FuzzyControl.h
 * @author mahx(MContour) m-contour@qq.com
 * @brief Fuzzy logic controller realization
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022 Fuzzy Limited. All rights reserved.
 * 
 */

#ifndef __FUZZYCONTROL__H__
#define __FUZZYCONTROL__H__

#include "Base.h"
#include "Operation.h"
#include "Membership.h"
#include <memory>


#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

namespace fc {
    class Membership;
    class FuzzyController {
        public:
            FuzzyController(scalar err_max, scalar dev_err_max, scalar u_max);
            ~FuzzyController();
            static constexpr int N = 7;
        public:

            scalar algo();

            scalar algo(scalar goal, scalar curr);

            void setFuzzyRule(int8_t rule[N][N]);

            void set_err_param(scalar* err_param);

            void set_err_dev_param(scalar* err_dev_param);

            void setGoal(scalar goal);

            void setCurrent(scalar curr);

            void setResolution(int16_t resolution);

            void setMembershipType(memebershipType type);

        private:
            /**
             * @brief Fuzzy logic (AND) used in inference mechanism
             * @param [in] premise1 membership function returned premise term degree 1
             * @param [in] premise2 membership function returned premise term degree 2
             * @return quantification premise degree
             */
            scalar _fuzzyUnion(scalar premise1, scalar premise2);

            /**
             * @brief Fuzzify the input value using membership function
             * @param [in] input fuzzification target
             * @param [in] type membership function type, Triangle, Rectangle, Trapzoid, Gaussian
             * @param [in] scale expand or narrow membership function basic value with a specific step
             * @return premise degree and index of effective membership
             */
            std::vector<std::pair<scalar, uint8_t>> _fuzzify(memebershipType type, scalar input, scalar* param);

            
            /**
             * @brief Defuzzify and inference the conclusion for action using `COG`
             * @param input receive data from `_fuzzify()` function
             * @return output
             */
            scalar _defuzzify(std::vector<std::pair<scalar, uint8_t>> &input1, std::vector<std::pair<scalar, uint8_t>> &input2);


            scalar _getArea(scalar chopOff_premise);

        private:
            scalar m_err;
            scalar m_err_last;
            scalar m_dev_err;
            scalar m_err_max;
            scalar m_dev_err_max;
            scalar m_u_max;

            scalar m_goal;
            scalar m_curr;
            
            scalar Kp_e;
            scalar Kd_e;
            scalar Kp_u;

            scalar* m_err_param;
            scalar* m_err_dev_param;

            int8_t m_ruleMatrix[N][N] = {{NB,NB,NM,NM,NS,ZO,ZO},
                                         {NB,NB,NM,NS,NS,ZO,PS},
                                         {NM,NM,NM,NS,ZO,PS,PS},
                                         {NM,NM,NS,ZO,PS,PM,PM},
                                         {NS,NS,ZO,PS,PS,PM,PM},
                                         {NS,ZO,PS,PM,PM,PM,PB},
                                         {ZO,ZO,PM,PM,PM,PB,PB}};
            std::shared_ptr<fc::Membership> m_memFunc;
            memebershipType membershipFuncSel = memebershipType::Triangle;
            int16_t m_resolution;
    };

    enum class memebershipType {
        Rectangle,
        Triangle,
        Trapezoid,
        Gaussian,
    };
    
}


#endif  //!__FUZZYCONTROL__H__