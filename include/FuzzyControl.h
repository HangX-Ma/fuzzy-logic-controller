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

    typedef std::pair<scalar, int8_t> fuzzifyPackType;
    typedef std::pair<scalar, scalar> centroidPackType;
    typedef std::vector<scalar> scalar_vec;
    enum class membershipType {
        Rectangle,
        Triangle,
        Trapezoid,
        Gaussian,
    };

    enum class ErrorStatus {
        ERROR = 0,
        SUCCESS = !ERROR,
    };

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

            void set_err_param(std::vector<scalar>& err_param);

            void set_err_dev_param(std::vector<scalar>& err_dev_param);

            void set_u_param(std::vector<scalar>& u_param);

            void setMembershipType_err(membershipType type);

            void setMembershipType_err_dev(membershipType type);

            void setMembershipType_u(membershipType type);

            void setGoal(scalar goal);

            void setCurrent(scalar curr);
            
            void setParam_K(scalar Kp_e, scalar Kd_e, scalar Kp_u, scalar Ki_e, scalar K_sat);

            void showInfo();

            /**
             * @brief Area size calculation resolution
             * @param [in] resolution 16 bits integer type
             */
            void setResolution(int16_t resolution);

            /**
             * @brief Set the Membership Type object
             * @param [in] type membership function type, Triangle, Rectangle, Trapzoid, Gaussian
             */
            void setMembershipType(membershipType type);

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
             * @param [in] param universal discourse range for current membership function 
             * @return premise degree and index of effective membership
             */
            std::vector<fuzzifyPackType> _fuzzify(membershipType type, scalar input, std::vector<scalar>& param);

            scalar _inference_and_defuzzify(membershipType                type,
                                            std::vector<fuzzifyPackType>& err_pack, 
                                            std::vector<fuzzifyPackType>& err_dev_pack,
                                            scalar_vec&                   param);

            /**
             * @brief get the centroid parameters for defuzzification process
             * @param [in] chopOff_premise Minimum operation is used to get the premise degree
             * @param [in] index indicate which universal discourse is determined
             * @param [in] param universal discourse range for current membership function 
             * @return num, den
             */
            centroidPackType _getCentroidParam(membershipType type, scalar chopOff_premise, uint8_t index, std::vector<scalar>& param);

            /**
             * @brief Check membership function type required parameters number whether equal to input parameter number or not.
             * @param type membership function type
             * @param param membership function parameter list
             * @retval ERROR 
             * @retval SUCCESS 
             */
            ErrorStatus _paramCheck(membershipType type, std::vector<scalar>& param);

        private:
            scalar m_err;
            scalar m_err_last;
            scalar m_err_dev; // derivative e
            scalar m_err_int; // integration e
            scalar m_fb_u; // feedback delta u that prevents saturating integration 

            scalar m_err_max;
            scalar m_err_dev_max;
            scalar m_u_max; // output maximum limitation

            scalar m_goal;
            scalar m_curr;

            scalar Ki_e;
            scalar Kp_e;
            scalar Kd_e;
            scalar Kp_u;
            scalar K_sat;

             // This parameters are consistant in membership functions' input parameters times N
            std::vector<scalar> m_err_param;
            std::vector<scalar> m_err_dev_param;
            std::vector<scalar> m_u_param;

            int8_t m_ruleMatrix[N][N] = {{NB,NB,NM,NM,NS,ZO,ZO},
                                         {NB,NB,NM,NS,NS,ZO,PS},
                                         {NM,NM,NM,NS,ZO,PS,PS},
                                         {NM,NM,NS,ZO,PS,PM,PM},
                                         {NS,NS,ZO,PS,PS,PM,PM},
                                         {NS,ZO,PS,PM,PM,PM,PB},
                                         {ZO,ZO,PM,PM,PM,PB,PB}};
            std::shared_ptr<fc::Membership> m_memFunc;
            membershipType m_err_membershipFuncSel     = membershipType::Triangle;
            membershipType m_err_dev_membershipFuncSel = membershipType::Triangle;
            membershipType m_u_membershipFuncSel       = membershipType::Triangle;

            int16_t m_resolution;
    };
}


#endif  //!__FUZZYCONTROL__H__