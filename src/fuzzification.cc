#include "fuzzy/utils.h"
#include "fuzzy/membership.h"
#include "fuzzy/fuzzification.h"
#include "spdlog/spdlog.h"

namespace fc
{

Fuzzification::Fuzzification(const std::string &name, scalar bound)
    : name(name), factor_(0), bound_(bound)
{
    membership_ = std::make_shared<Membership>(name);
}

void Fuzzification::init(const scalar bound, const bool reverse, const MembershipType type,
                         const scalar input_params[], const uint8_t params_num)
{
    membership_->setMembershipParam(type, input_params, params_num);
    bound_ = bound;
    factor_ = static_cast<scalar>(membership_->getDiscourseSize() / 2) / bound_;
    if (reverse) {
        factor_ = 1.0 / factor_;
    }

    state = FuzzyProcess::FuzzyInit;
}

void Fuzzification::fuzzify(scalar input)
{
    std::optional<scalar> premise;

    premise_pairs_.clear();

    for (size_t idx = 0; idx < membership_->getDiscourseSize(); idx++) {
        premise = membership_->calculate(input, membership_->getParamSet(idx));
        if (premise != std::nullopt && fabs(premise.value()) >= fc::eps) {
            premise_pairs_.emplace_back(premise.value(), idx);
            spdlog::debug("[Fuzzify] {:<2s} - premise: {:> 5.3f}, discourse: {}, input: {:< 8.3f}",
                          getName(), premise.value(), idx, input);
        }
    }
}

void Fuzzification::setFactor(const scalar ratio, const bool reverse)
{
    if (state != FuzzyProcess::FuzzyInit) {
        spdlog::warn("[Fuzzification] please init first");
        return;
    }

    scalar input_ratio = ratio > fc::eps ? ratio : 0.1;
    factor_ = input_ratio * static_cast<scalar>(membership_->getDiscourseSize() / 2) / bound_;
    if (reverse) {
        factor_ = 1 / factor_;
    }
}

void Fuzzification::setBound(scalar bound)
{
    if (state != FuzzyProcess::FuzzyInit) {
        spdlog::warn("[Fuzzification] please init first");
        return;
    }

    if (bound < -eps) {
        spdlog::warn("[Fuzzification] 'bound' needs to be positive");
        return;
    }

    bound_ = bound;
}

const std::string &Fuzzification::getName() const { return name; }

scalar Fuzzification::getFactor() const { return factor_; }

scalar Fuzzification::getBound() const { return bound_; }

} // namespace fc