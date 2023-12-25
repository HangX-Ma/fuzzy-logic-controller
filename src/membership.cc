#include "fmt/core.h"
#include "fuzzy/membership.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace fc {


Membership::Membership(std::string name, scalar height)
    : height_(height),
      name_(std::move(name)),
      type_(MembershipType::None),
      discourse_size_(0) {}


std::optional<scalar> Membership::calculate(const scalar input, const std::vector<scalar>& param_set) {
    scalar ret;

    switch (type_) {
    case MembershipType::Triangle:
        ret = triangle(input, param_set.at(0), param_set.at(1), param_set.at(2));
        break;
    case MembershipType::Trapezoid:
        ret = trapezoid(input, param_set.at(0), param_set.at(1), param_set.at(2), param_set.at(3));
        break;
    case MembershipType::Gaussian:
        ret = gaussian(input, param_set.at(0), param_set.at(1));
        break;
    case MembershipType::None:
        /* fall through */
    default:
        throw std::invalid_argument("[membership error] set membership type first");
    }

    if (ret != fc::nan && ret != fc::inf) {
        return ret;
    }

    return std::nullopt;
}

Range Membership::calculateRange(const MembershipType type, const std::vector<scalar>& param_set) {
    Range range;
    scalar mean;
    scalar derivation;
    switch (type) {
    case MembershipType::Triangle:
        range = Range{param_set.at(0), param_set.at(2)};
        break;
    case MembershipType::Trapezoid:
        range = Range{param_set.at(0), param_set.at(3)};
        break;
    case MembershipType::Gaussian:
        mean = param_set.at(0);
        derivation = param_set.at(1);
        range = Range{-3.0 * derivation + mean, 3.0 * derivation + mean};
        break;
    case MembershipType::None:
        /* fall through */
    default:
        throw std::invalid_argument("[membership error] set membership type first");
    }

    if (range.first < minimum_) {
        minimum_ = range.first;
    }

    if (range.second > maximum_) {
        maximum_ = range.second;
    }

    return range;
}

void Membership::setMembershipParam(const MembershipType type,
                                    const scalar input_params[],
                                    uint8_t params_num)
{
    auto size = static_cast<uint8_t>(type);
    std::vector<scalar> param_set;

    if (params_num % size != 0) {
        warnmsgln("[membership error] membership type: %d, param number: %d", size, params_num);
        throw std::length_error("[membership error] parameters number mismatches");
    }

    type_ = type;
    discourse_size_ = params_num / size;
    minimum_ = std::numeric_limits<scalar>::max();
    maximum_ = std::numeric_limits<scalar>::min();

    for (uint8_t idx = 1; idx <= params_num; idx++) {
        param_set.emplace_back(input_params[idx - 1]);
        // store the params set by group according to membership function required params size
        if (idx % size == 0) {
            std::string s = fmt::format("[membership {}] id:{}, (",this->getName().data() , idx / size);
            for (auto [i, elem] = std::tuple{1, param_set.begin()}; elem != param_set.end(); i++, elem++) {
                s.append(fmt::format("{:.2f}",*elem));
                if (i == size) {
                    s.append(")\n");
                } else {
                    s.append(", ");
                }
            }
            dbgmsg("%s", s.c_str());
            params_.emplace_back(param_set);
            params_range_.emplace_back(calculateRange(type, param_set));
            param_set.clear();
        }
    }
}


scalar Membership::triangle(scalar x, scalar vertexA, scalar vertexB, scalar vertexC) const {
    if (Operation::isNaN(x)) {
        return fc::nan;
    }

    if (Operation::isLessThan(x, vertexA) || Operation::isGreaterThan(x, vertexC)) {
        return height_ * 0.0;

    } // outside the membership area

    if (Operation::isEqual(x, vertexB)) {
        return height_ * 1.0;
    }

    if (Operation::isLessThan(x, vertexB)) {
        if (fabs(vertexA) == fc::inf) {
            return height_ * 1.0;
        }
        return height_ * (x - vertexA) / (vertexB - vertexA);
    }

    if (Operation::isGreaterThan(x, vertexB)) {
        if (fabs(vertexC) == fc::inf) {
            return height_ * 1.0;
        }
        return height_ * (vertexC - x) / (vertexC - vertexB);
    }

    return fc::nan;
}


scalar Membership::trapezoid(scalar x, scalar vertexA, scalar vertexB, scalar vertexC, scalar vertexD) const {
    if (Operation::isNaN(x)) {
        return fc::nan;
    }

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

MembershipType Membership::getType(void) {
    return type_;
}


const std::string& Membership::getName(void) {
    return name_;
}

scalar Membership::getMinimum(void) {
    return minimum_;
}

scalar Membership::getMaximum(void) {
    return maximum_;
}

}