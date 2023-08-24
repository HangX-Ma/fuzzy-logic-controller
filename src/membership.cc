#include "fuzzy/membership.h"
#include <cmath>
#include <stdexcept>

using namespace fc;

Membership::Membership(std::string name, scalar height)
    : height_(height),
      name_(name),
      type_(membershipType::None),
      discourse_size_(0) {}

Membership::~Membership() {}

std::optional<scalar> Membership::calculate(const scalar input, const std::vector<scalar>& param_set) {
    scalar ret;

    switch (type_) {
        case membershipType::Triangle:
            ret = triangle(input, param_set.at(0), param_set.at(1), param_set.at(2));
            break;
        case membershipType::Trapezoid:
            ret = trapezoid(input, param_set.at(0), param_set.at(1), param_set.at(2), param_set.at(3));
            break;
        case membershipType::Gaussian:
            ret = gaussian(input, param_set.at(0), param_set.at(1));
            break;
        case membershipType::None:
            /* fall through */
        default:
            throw std::invalid_argument("[membership error] set membership type first");
    }

    if (ret != fc::nan) {
        return ret;
    }

    return std::nullopt;
}

Range Membership::calculateRange(const membershipType type, const std::vector<scalar>& param_set) {
    Range range;
    scalar derivation;
    switch (type) {
        case membershipType::Triangle:
            range = Range{param_set.at(0), param_set.at(2)};
            break;
        case membershipType::Trapezoid:
            range = Range{param_set.at(0), param_set.at(3)};
            break;
        case membershipType::Gaussian:
            derivation = param_set.at(1);
            range = Range{-3.0 * derivation, 3.0 * derivation};
            break;
        case membershipType::None:
            /* fall through */
        default:
            throw std::invalid_argument("[membership error] set membership type first");
    }

    return range;
}

void Membership::setMembershipParam(const membershipType type,
                                    const scalar input_params[],
                                    uint8_t params_num)
{
    uint8_t size = static_cast<uint8_t>(type);
    std::vector<scalar> param_set;

    if (params_num % size != 0) {
        printf("[membership error] membership type: %d, param number: %d\n", size, params_num);
        throw std::length_error("[membership error] parameters number mismatches");
    }

    type_ = type;
    discourse_size_ = params_num / size;

    for (uint8_t idx = 1; idx <= params_num; idx++) {
        param_set.emplace_back(input_params[idx - 1]);
        // store the params set by group according to membership function required params size
        if (idx % size == 0) {
            dbgmsg("[membership %s] id: %d, (%f, %f, %f)",
                this->getName().data() , idx / size, param_set.at(0), param_set.at(1), param_set.at(2));
            params_.emplace_back(param_set);
            params_range_.emplace_back(calculateRange(type, param_set));
            param_set.clear();
        }
    }
}


scalar Membership::triangle(scalar x, scalar vertexA, scalar vertexB, scalar vertexC) const {
    if (Operation::isNaN(x)) return fc::nan;

    if (Operation::isLessThan(x, vertexA) || Operation::isGreaterThan(x, vertexC)) {
        return height_ * 0.0;

    } // outside the membership area

    if (Operation::isEqual(x, vertexB)) {
        return height_ * 1.0;
    }

    if (Operation::isLessThan(x, vertexB)) {
        if (vertexA == -fc::inf) {
            return height_ * 1.0;
        }
        return height_ * (x - vertexA) / (vertexB - vertexA);
    }
    if (vertexC == fc::inf) {
        return height_ * 1.0;
    }

    return height_ * (vertexC - x) / (vertexC - vertexB);
}


scalar Membership::trapezoid(scalar x, scalar vertexA, scalar vertexB, scalar vertexC, scalar vertexD) const {
    if (Operation::isNaN(x)) return fc::nan;

    if (Operation::isLessThan(x, vertexA) || Operation::isGreaterThan(x, vertexD)) {
        return height_ * 0.0;
    } // outside the membership area

    if (Operation::isLessThan(x, vertexB)) {
        if (vertexA == -fc::inf) {
            return height_ * 1.0;
        }
        return height_ * Operation::min(scalar(1.0), (x - vertexA) / (vertexB - vertexA));
    }

    if (Operation::isLessThan(x, vertexC)) {
        return height_ * 1.0;
    }

    if (Operation::isLessThan(x, vertexD)) {
        if (vertexD == fc::inf) {
            return height_ * 1.0;
        }
        return height_ * (vertexD - x) / (vertexD - vertexC);
    }

    if (vertexD == fc::inf) {
        return height_ * 1.0;
    }

    return height_ * 0.0;
}


scalar Membership::gaussian(scalar x, scalar mean, scalar standardDeviation) const {
    if (Operation::isNaN(x)) return fc::nan;

    return height_ * std::exp((-(x - mean) * (x - mean)) / (2.0 * standardDeviation * standardDeviation));
}


void Membership::setHeight(scalar height) {
    height_ = height;
}


scalar Membership::getHeight() const {
    return height_;
}


size_t Membership::getDiscourseSize(void) {
    return discourse_size_;
}

const std::vector<scalar> Membership::getParamSet(size_t discourse_id) {
    return params_.at(discourse_id);
}

const Range Membership::getRange(size_t discourse_id) {
    return params_range_.at(discourse_id);
}

membershipType Membership::getType(void) {
    return type_;
}


const std::string& Membership::getName(void) {
    return name_;
}