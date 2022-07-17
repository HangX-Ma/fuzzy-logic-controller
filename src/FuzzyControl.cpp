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
    FuzzyController::FuzzyController(scalar err_max, scalar err_dev_max, scalar u_max) 
        : m_err_max(err_max), m_err_dev_max(err_dev_max), m_u_max(u_max), m_goal(0), m_curr(0),
        m_resolution(100), m_err_param(nullptr), m_err_dev_param(nullptr), m_u_param(nullptr) {

        m_err      = m_goal - m_curr;
        m_err_last = 0;
        m_err_dev  = m_err - m_err_last;

        Kp_e      = (N/2)/m_err_max;
        Kd_e      = (N/2)/m_err_dev_max;
        Kp_u      = m_u_max/(N/2);

        m_memFunc  = std::make_shared<fc::Membership>();
    }

    FuzzyController::~FuzzyController() {}

    scalar FuzzyController::algo() {
        if (m_err_param == nullptr || m_err_dev_param == nullptr || m_u_param == nullptr) {
            throw "Please set err_param and err_dev_param first";

            return fc::nan;
        }

        scalar u;

        m_err     = m_goal - m_curr;
        m_err_dev = m_err - m_err_last;

        m_err     = Kp_e * m_err;
        m_err_dev = Kd_e * m_err_dev;

        // printf("Kp_e=%f, Kd_e=%f, Kp_u=%f\n", Kp_e, Kd_e, Kp_u);
        // printf("err=%f, err_dev=%f\n", m_err, m_err_dev);

        std::vector<std::pair<scalar, uint8_t>> errFuzzified = this->_fuzzify(m_err, m_err_param);
        std::vector<std::pair<scalar, uint8_t>> err_devFuzzified = this->_fuzzify(m_err_dev, m_err_dev_param);

        auto err_pack = this->_defuzzify(errFuzzified, m_u_param);
        auto err_dev_pack = this->_defuzzify(err_devFuzzified, m_u_param);

        u = (err_pack.first + err_dev_pack.first) / (err_pack.second + err_dev_pack.second);

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

    std::vector<std::pair<scalar, uint8_t>> FuzzyController::_fuzzify(scalar input, scalar* param) {
        std::vector<std::pair<scalar, uint8_t>> premisePairIndex;
        scalar premise;
        for (int i = 0; i < N; i++) {
            if (membershipFuncSel == membershipType::Triangle) {
                premise = m_memFunc->Triangle(input, param[i*3], param[i*3+1], param[i*3+2]);
            } else if (membershipFuncSel == membershipType::Trapezoid) {
                premise = m_memFunc->Trapezoid(input, param[i*4], param[i*4+1], param[i*4+2], param[i*4+3]);
            } else if (membershipFuncSel == membershipType::Rectangle) {
                premise = m_memFunc->Rectangle(input, param[i*2], param[i*2+1]);
            } else if (membershipFuncSel == membershipType::Gaussian) {
                premise = m_memFunc->Gaussian(input, param[i*2], param[i*2+1]);
            }

            // printf("_fuzzify: epoch=%d, premise=%f\n", i, premise);
            if (std::abs(premise) >= fc::eps) {
                premisePairIndex.push_back(std::make_pair(premise, i));
            }
        }

        return premisePairIndex;
    }


    std::pair<scalar, scalar> FuzzyController::_defuzzify(std::vector<std::pair<scalar, uint8_t>> &pack, scalar* param) {
        int8_t index_size = static_cast<int8_t>(pack.size());
        scalar num = 0, den = 0;

        for (int8_t i = 0; i < index_size; i++) {
            auto centroid_param = this->_getCentroidParam(pack.at(i).first, pack.at(i).second, param);
            num += centroid_param.first;
            den += centroid_param.second;
        }

        return std::make_pair(num, den);
    }

    std::pair<scalar, scalar> FuzzyController::_getCentroidParam(scalar chopOff_premise, uint8_t index, scalar* param) {
        uint8_t M = 0;
        if (membershipFuncSel == membershipType::Triangle) M = 3;
        if (membershipFuncSel == membershipType::Trapezoid) M = 4;
        if (membershipFuncSel == membershipType::Rectangle || membershipFuncSel == membershipType::Gaussian) M = 2;

        scalar dx = (param[(index+1)*M-1] - param[index*M]) / m_resolution;
        scalar area = 0, x = param[0], xCentroid = 0;
        scalar curr_premise;

        for (int i = 0; i < m_resolution; i++) {
            x = param[index*M] + (i+0.5) * dx;
            if (membershipFuncSel == membershipType::Triangle) {
                curr_premise = m_memFunc->Triangle(x, param[index*M], param[index*M+1], param[index*M+2]);
            } else if (membershipFuncSel == membershipType::Trapezoid) {
                curr_premise = m_memFunc->Trapezoid(x, param[index*M*4], param[index*M*4+1], param[index*M*4+2], param[index*M*4+3]);
            } else if (membershipFuncSel == membershipType::Rectangle) {
                curr_premise = m_memFunc->Rectangle(x, param[index*M*2], param[index*M*2+1]);
            } else if (membershipFuncSel == membershipType::Gaussian) {
                curr_premise = m_memFunc->Gaussian(x, param[index*M*2], param[index*M*2+1]);
            }

            scalar y = Operation::min(curr_premise, chopOff_premise);

            xCentroid += y * x;
            area += y;
        } // COG method: For convience, multiply dx with y curr_premise is ignored.

        return std::make_pair(xCentroid, area);
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


    void FuzzyController::set_u_param(scalar* u_param) {
        this->m_u_param = u_param;
    }

    void FuzzyController::showInfo() {
        std::cout << ANSI_COLOR_BLUE << 
        "------------ Information of fuzzy logic controller ------------" 
        << ANSI_COLOR_RESET << std::endl;
        
        printf("Universal discourse [err]: [%f, %f]\n", -this->m_err_max, this->m_err_max);
        printf("Universal discourse [err_dev]: [%f, %f]\n", -this->m_err_dev_max, this->m_err_dev_max);
        printf("Universal discourse [u]: [%f, %f]\n", -this->m_u_max, this->m_u_max);
        printf("Kp_e=%f, Kd_e=%f, Kp_u=%f\n", this->Kp_e, this->Kd_e, this->Kp_u);
    }

}
