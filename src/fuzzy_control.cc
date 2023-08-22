#include "fuzzy/fuzzy_control.h"

#include <iostream>
#include <exception>

namespace fc {
FuzzyController::FuzzyController(scalar err_max, scalar err_dev_max, scalar u_max)
    : m_err_max(err_max), m_err_dev_max(err_dev_max), m_u_max(u_max), m_goal(0), m_curr(0),
    m_resolution(100) {

    m_err      = m_goal - m_curr;
    m_err_last = 0;
    m_err_dev  = m_err - m_err_last;
    m_err_int  = 0;
    m_fb_u     = 0;

    Kp_e      = (N/2)/m_err_max;
    Kd_e      = (N/2)/m_err_dev_max;
    Kp_u      = m_u_max/(N/2);
    // default PD system
    K_sat    = 0;
    Ki_e     = 0;

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
    scalar u_sat; // saturated output u

    m_err      = m_goal - m_curr;
    m_err_dev  = m_err - m_err_last;

    // clear integral value
    if ((m_err > 0 && m_err_last < 0) || (m_err < 0 && m_err_last > 0)) {
        m_err_int = 0;
    }

    m_err      = Kp_e * m_err;
    m_err_dev  = Kd_e * m_err_dev;
    m_err_int += Ki_e * m_err - m_fb_u;

    // input limitation
    m_err      = m_err > m_err_max ? m_err_max : (m_err < -m_err_max ? -m_err_max : m_err);
    m_err_dev  = m_err_dev > m_err_dev_max ? m_err_dev_max : (m_err_dev < -m_err_dev_max ? -m_err_dev_max : m_err_dev);


    // printf("Kp_e=%f, Kd_e=%f, Kp_u=%f\n", Kp_e, Kd_e, Kp_u);
    // printf("err=%f, err_dev=%f\n", m_err, m_err_dev);

    std::vector<fuzzifyPackType> errFuzzified =
                            this->_fuzzify(m_err_membershipFuncSel, m_err, m_err_param);
    std::vector<fuzzifyPackType> err_devFuzzified =
                            this->_fuzzify(m_err_dev_membershipFuncSel, m_err_dev, m_err_dev_param);

    u = this->_inference_and_defuzzify(m_u_membershipFuncSel, errFuzzified, err_devFuzzified, m_u_param);

    u     = Kp_u * u + m_err_int;
    u_sat = u;

    if (u_sat > m_u_max)  u_sat =  m_u_max;
    if (u_sat < -m_u_max) u_sat = -m_u_max;

    // update last error and feedback u that prevents saturating integration
    m_fb_u = (u - u_sat) * K_sat;
    m_err_last = m_err;

    return u_sat;
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
    for (int8_t i = 0; i < N; i++) {
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


scalar FuzzyController::_inference_and_defuzzify(
    membershipType type,
    std::vector<fuzzifyPackType>& err_pack,
    std::vector<fuzzifyPackType>& err_dev_pack,
    scalar_vec&                   param)
{
    uint8_t err_pack_size = err_pack.size();
    uint8_t err_dev_pack_size = err_dev_pack.size();
    centroidPackType centroid_param;
    scalar chopOff_premise;
    int8_t u_index;
    scalar num = 0; scalar den = 0;

    for (uint8_t m = 0; m < err_pack_size; m++) {
        for (uint8_t n = 0; n < err_dev_pack_size; n++) {
            // minimum operation. This operation is used to get the chop off premise value
            chopOff_premise = Operation::min(err_pack.at(m).first, err_dev_pack.at(n).first);
            // inference process. This process will generate the output field range. N/2 is the compensation
            // of the index value.
            u_index         = m_ruleMatrix[err_pack.at(m).second][err_dev_pack.at(n).second] + N/2;
            centroid_param  = this->_getCentroidParam(type, chopOff_premise, u_index, param);

            num += centroid_param.first;
            den += centroid_param.second;
        }
    }

    return num/den;
}

centroidPackType FuzzyController::_getCentroidParam(
        membershipType type,
        scalar         chopOff_premise,
        uint8_t        index,
        scalar_vec&    param)
{
    uint8_t M = 0;
    if (type == membershipType::Triangle) M = 3;
    if (type == membershipType::Trapezoid) M = 4;
    if (type == membershipType::Rectangle || type == membershipType::Gaussian) M = 2;

    scalar dx = (param[(index+1)*M-1] - param[index*M]) / m_resolution;
    scalar area = 0, xCentroid = 0;
    scalar x, x_start = param[index*M];
    scalar curr_premise;

    for (int i = 0; i < m_resolution; i++) {
        x = x_start + (i+0.5) * dx;
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

void FuzzyController::setParam_K(scalar Kp_e, scalar Kd_e, scalar Kp_u, scalar Ki_e, scalar K_sat) {
    this->Ki_e  = Ki_e;
    this->Kp_e  = Kp_e;
    this->Kd_e  = Kd_e;
    this->Kp_u  = Kp_u;
    this->K_sat = K_sat;
}


void FuzzyController::showInfo() {
    std::cout << ANSI_COLOR_BLUE << 
    "------------ Information of fuzzy logic controller ------------" 
    << ANSI_COLOR_RESET << std::endl;
    printf("Universal discourse [err]: [%f, %f]\n", -this->m_err_max, this->m_err_max);
    printf("Universal discourse [err_dev]: [%f, %f]\n", -this->m_err_dev_max, this->m_err_dev_max);
    printf("Universal discourse [u]: [%f, %f]\n", -this->m_u_max, this->m_u_max);
    printf("Kp_e=%f, Ki_e=%f, Kd_e=%f, Kp_u=%f, K_sat=%f\n", this->Kp_e,this->Ki_e, this->Kd_e, this->Kp_u, this->K_sat);
}

}
