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
#include <exception>
#include <cstdio>

namespace fc {
    FuzzyController::FuzzyController(scalar err_max, scalar err_dev_max, scalar u_max) 
        : m_err_max(err_max), m_err_dev_max(err_dev_max), m_u_max(u_max), m_goal(0), m_curr(0),
        m_resolution(100) {

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
        if (m_err_param.size() == 0 || m_err_dev_param.size() == 0 || m_u_param.size() == 0) {
            printf(ANSI_COLOR_RED "Please set err_param, err_dev_param and m_u_param first" ANSI_COLOR_RESET);
            throw ("No parameters");

            return fc::nan;
        }

        if (this->_paramCheck(m_err_membershipFuncSel, m_err_param) == ErrorStatus::ERROR ||
            this->_paramCheck(m_err_dev_membershipFuncSel, m_err_dev_param) == ErrorStatus::ERROR ||
            this->_paramCheck(m_u_membershipFuncSel, m_u_param) == ErrorStatus::ERROR) {

            printf(ANSI_COLOR_RED "Input parameter number is not equal to what membership function required.\
                \nTriangle [Nx3]\nTrapezoid [Nx4]\nRectangle [Nx2]\nGaussian [Nx2]\n" ANSI_COLOR_RESET);
            throw ("data size error");

            return fc::nan;
        }

        scalar u;

        m_err     = m_goal - m_curr;
        m_err_dev = m_err - m_err_last;

        m_err     = Kp_e * m_err;
        m_err_dev = Kd_e * m_err_dev;

        // input limitation
        m_err     = m_err > m_err_max ? m_err_max : (m_err < -m_err_max ? -m_err_max : m_err);
        m_err_dev = m_err_dev > m_err_dev_max ? m_err_dev_max : (m_err_dev < -m_err_dev_max ? -m_err_dev_max : m_err_dev);


        // printf("Kp_e=%f, Kd_e=%f, Kp_u=%f\n", Kp_e, Kd_e, Kp_u);
        // printf("err=%f, err_dev=%f\n", m_err, m_err_dev);

        std::vector<fuzzifyPackType> errFuzzified = 
                                this->_fuzzify(m_err_membershipFuncSel, m_err, m_err_param);
        std::vector<fuzzifyPackType> err_devFuzzified = 
                                this->_fuzzify(m_err_dev_membershipFuncSel, m_err_dev, m_err_dev_param);

        defuzzifyPackType err_pack = this->_defuzzify(m_u_membershipFuncSel, errFuzzified, m_u_param);
        defuzzifyPackType err_dev_pack = this->_defuzzify(m_u_membershipFuncSel, err_devFuzzified, m_u_param);

        u = (err_pack.first + err_dev_pack.first) / (err_pack.second + err_dev_pack.second);

        u = Kp_u * u;

        if (u > m_u_max)  u = m_u_max;
        if (u < -m_u_max) u = -m_u_max;

        m_err_last = m_err;

        return u;
    }

    scalar FuzzyController::algo(scalar goal, scalar curr) {
        this->setGoal(goal);
        this->setCurrent(curr);
        
        return this->algo();
    }

    std::vector<fuzzifyPackType> FuzzyController::_fuzzify(membershipType type, 
                                                           scalar         input, 
                                                           scalar_vec&    param) {
        std::vector<fuzzifyPackType> premisePairIndex;
        scalar premise;
        for (uint8_t i = 0; i < N; i++) {
            if (type == membershipType::Triangle) {
                premise = m_memFunc->Triangle(input, param[i*3], param[i*3+1], param[i*3+2]);
            } else if (type == membershipType::Trapezoid) {
                premise = m_memFunc->Trapezoid(input, param[i*4], param[i*4+1], param[i*4+2], param[i*4+3]);
            } else if (type == membershipType::Rectangle) {
                premise = m_memFunc->Rectangle(input, param[i*2], param[i*2+1]);
            } else if (type == membershipType::Gaussian) {
                premise = m_memFunc->Gaussian(input, param[i*2], param[i*2+1]);
            }
            if (std::abs(premise) >= fc::eps) {
                premisePairIndex.push_back(std::make_pair(premise, i));
            }
        } // This `for` cycle is designed to find which linguistic discourse the input locates at. 
          // If `premise` is not zero, save it in `premisePairIndex` vector.

        return premisePairIndex;
    }


    defuzzifyPackType FuzzyController::_defuzzify(membershipType                type, 
                                                  std::vector<fuzzifyPackType>& pack, 
                                                  scalar_vec&                   param) {
        int8_t index_size = static_cast<int8_t>(pack.size());
        scalar num = 0, den = 0;

        for (int8_t i = 0; i < index_size; i++) {
            auto centroid_param = this->_getCentroidParam(type, pack.at(i).first, pack.at(i).second, param);
            num += centroid_param.first;
            den += centroid_param.second;
        }

        return std::make_pair(num, den);
    }

    defuzzifyPackType FuzzyController::_getCentroidParam(membershipType type, 
                                                         scalar         chopOff_premise, 
                                                         uint8_t        index, 
                                                         scalar_vec&    param) {
        uint8_t M = 0;
        if (type == membershipType::Triangle) M = 3;
        if (type == membershipType::Trapezoid) M = 4;
        if (type == membershipType::Rectangle || type == membershipType::Gaussian) M = 2;

        scalar dx = (param[(index+1)*M-1] - param[index*M]) / m_resolution;
        scalar area = 0, x = param[0], xCentroid = 0;
        scalar curr_premise;

        for (int i = 0; i < m_resolution; i++) {
            x = param[index*M] + (i+0.5) * dx;
            if (type == membershipType::Triangle) {
                curr_premise = m_memFunc->Triangle(x, param[index*M], param[index*M+1], param[index*M+2]);
            } else if (type == membershipType::Trapezoid) {
                curr_premise = m_memFunc->Trapezoid(x, param[index*M], param[index*M+1], param[index*M+2], param[index*M+3]);
            } else if (type == membershipType::Rectangle) {
                curr_premise = m_memFunc->Rectangle(x, param[index*M], param[index*M+1]);
            } else if (type == membershipType::Gaussian) {
                curr_premise = m_memFunc->Gaussian(x, param[index*M], param[index*M+1]);
            }

            scalar y = Operation::min(curr_premise, chopOff_premise);

            xCentroid += y * x;
            area      += y;
        } // COG method: For convience, multiply dx with y curr_premise is ignored.

        return std::make_pair(xCentroid, area);
    }


    ErrorStatus FuzzyController::_paramCheck(membershipType type, scalar_vec& param) {
        if (type == membershipType::Triangle) {
            if (param.size() != N*3) {
                return ErrorStatus::ERROR;
            }
        }
        if (type == membershipType::Trapezoid) {
            if (param.size() != N*4) {
                return ErrorStatus::ERROR;
            }
        }
        if (type == membershipType::Rectangle || type == membershipType::Gaussian) {
            if (param.size() != N*2) {
                return ErrorStatus::ERROR;
            }
        }

        return ErrorStatus::SUCCESS;
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


    void FuzzyController::setMembershipType_err(membershipType type) {
        this->m_err_dev_membershipFuncSel = type;
    }


    void FuzzyController::setMembershipType_err_dev(membershipType type) {
        this->m_err_membershipFuncSel = type;
    }


    void FuzzyController::setMembershipType_u(membershipType type) {
        this->m_u_membershipFuncSel = type;
    }


    void FuzzyController::set_err_param(scalar_vec& err_param) {
        this->m_err_param = err_param;
    }


    void FuzzyController::set_err_dev_param(scalar_vec& err_dev_param) {
        this->m_err_dev_param = err_dev_param;
    }


    void FuzzyController::set_u_param(scalar_vec& u_param) {
        this->m_u_param = u_param;
    }

    void FuzzyController::setParam_K(scalar Kp_e, scalar Kd_e, scalar Kp_u) {
        this->Kp_e = Kp_e;
        this->Kd_e = Kd_e;
        this->Kp_u = Kp_u;
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
