/**
 * @file FuzzyControl.cpp
 * @author mahx(MContour) m-contour@qq.com
 * @brief Fuzzy logic controller realization
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022 Fuzzy Limited. All rights reserved.
 * 
 */

#include "FuzzyControl.h"


namespace fc {
    FuzzyController::FuzzyController(scalar err_max, scalar dev_err_max, scalar u_max) 
    : m_err_max(err_max), m_dev_err_max(dev_err_max), m_u_max(u_max), m_goal(0), m_curr(0), m_resolution(100), m_err_param(nullptr), m_err_dev_param(nullptr) { 
        m_err      = m_goal - m_curr;
        m_err_last = 0;
        m_dev_err  = m_err - m_err_last;

        Kp_e      = (N/2)/m_err_max;
        Kd_e      = (N/2)/m_dev_err_max;
        Kp_u      = m_u_max/(N/2);

        m_memFunc  = std::make_shared<fc::Membership>();
    }

    FuzzyController::~FuzzyController() {}

    scalar FuzzyController::algo() {
        if (m_err_param == nullptr || m_err_dev_param == nullptr) {
            throw "Please set err_param and err_dev_param first";

            return fc::nan;
        }

        scalar u;

        m_err     = m_goal - m_curr;
        m_dev_err = m_err - m_err_last;

        m_err     = Kp_e * m_err;
        m_dev_err = Kd_e * m_dev_err;

        std::vector<std::pair<scalar, uint8_t>> errFuzzified = this->_fuzzify(membershipFuncSel, m_err, m_err_param);
        std::vector<std::pair<scalar, uint8_t>> err_devFuzzified = this->_fuzzify(membershipFuncSel, m_err, m_err_dev_param);

        u = this->_defuzzify(errFuzzified, err_devFuzzified);
        u = Kp_u * u;

        if (u > m_u_max) u = m_u_max;
        if (u < -m_u_max) u = -m_u_max;

        m_err_last = m_err;

        return u;
    }

    scalar FuzzyController::algo(scalar goal, scalar curr) {
        this->setGoal(goal);
        this->setCurrent(curr);
        
        return this->algo();
    }

    std::vector<std::pair<scalar, uint8_t>> FuzzyController::_fuzzify(membershipType type, scalar input, scalar* param) {
        std::vector<std::pair<scalar, uint8_t>> premisePairIndex;
        switch (type) {
            case membershipType::Triangle:
                for (int i = 0; i < N; i++) {
                    scalar premise = m_memFunc->Triangle(input, param[i*3], param[i*3+1], param[i*3+2]);
                    if (std::abs(premise) >= fc::eps) {
                        premisePairIndex.push_back(std::make_pair(premise, i));
                    }
                }
                break;
            case membershipType::Trapezoid:
                for (int i = 0; i < N; i++) {
                    scalar premise = m_memFunc->Trapezoid(input, param[i*4], param[i*4+1], param[i*4+2], param[i*4+3]);
                    if (std::abs(premise) >= fc::eps) {
                            premisePairIndex.push_back(std::make_pair(premise, i));
                    }
                }
                break;
            case membershipType::Rectangle:
                for (int i = 0; i < N; i++) {
                    scalar premise = m_memFunc->Rectangle(input, param[i*2], param[i*2+1]);
                    if (std::abs(premise) >= fc::eps) {
                            premisePairIndex.push_back(std::make_pair(premise, i));
                    }
                }
                break;
            case membershipType::Gaussian:
                for (int i = 0; i < N; i++) {
                    scalar premise = m_memFunc->Gaussian(input, param[i*2], param[i*2+1]);
                    if (std::abs(premise) >= fc::eps) {
                            premisePairIndex.push_back(std::make_pair(premise, i));
                    }
                }
                break;
        }

        return premisePairIndex;
    }


    scalar FuzzyController::_defuzzify(std::vector<std::pair<scalar, uint8_t>> &input1, std::vector<std::pair<scalar, uint8_t>> &input2) {
        int8_t index_size1 = static_cast<int8_t>(input1.size());
        int8_t index_size2 = static_cast<int8_t>(input2.size());
        scalar num = 0, den = 0;

        for (int8_t m = 0; m < index_size1; m++) {
            for (int8_t n = 0; n < index_size2; n++) {
                scalar area = this->_getArea(Operation::min(input1.at(m).first, input2.at(n).first));
                num += area * m_ruleMatrix[input1.at(m).second][input2.at(n).second];
                den += area;
            }
        }
        
        return num / den;
    }

    scalar FuzzyController::_getArea(scalar chopOff_premise) {
        scalar dx = 2 / m_resolution;
        scalar area = 0, x = -1.0;
        for (int i = 0; i < m_resolution; i++) {
            x = -1.0 + (i+0.5) * m_resolution;
            scalar curr_premise = m_memFunc->Triangle(x, -1.0, 0.0, 1.0);
            area += Operation::min(curr_premise, chopOff_premise) * dx;
        }

        return area;
    }

    void FuzzyController::setFuzzyRule(int8_t rule[N][N]) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                m_ruleMatrix[i][j] = rule[i][j];
            }
        }
    }


    void FuzzyController::setGoal(scalar goal) {
        this->m_goal = goal;
    }


    void FuzzyController::setCurrent(scalar curr) {
        this->m_curr = curr;
    }


    void FuzzyController::setResolution(int16_t resolution) {
        this->m_resolution = resolution;
    }


    void FuzzyController::setMembershipType(membershipType type) {
        this->membershipFuncSel = type;
    }


    void FuzzyController::set_err_param(scalar* err_param) {
        this->m_err_param = err_param;
    }


    void FuzzyController::set_err_dev_param(scalar* err_dev_param) {
        this->m_err_dev_param = err_dev_param;
    }

}
