#include "fuzzy/fuzzy_logic.h"
#include <iostream>
#include <cmath>

using namespace fc;

Fuzzification::Fuzzification(scalar bound)
    : factor_(0), bound_(bound)
{
    membership_ = std::make_unique<Membership>();
}

Fuzzification::~Fuzzification() {}

void Fuzzification::init(const scalar bound,
                         const bool reverse,
                         const membershipType type,
                         const scalar input_params[],
                         const uint8_t params_num)
{
    membership_->setMembershipParam(type, input_params, params_num);
    bound_ = bound;
    factor_ = bound_ / static_cast<scalar>(membership_->getDiscourseSize());
    if (reverse) factor_ = 1 / factor_;
}

void Fuzzification::setFactor(scalar factor) {
    factor_ = factor;
}

scalar Fuzzification::getFactor(void) {
    return factor_;
}

void Fuzzification::fuzzify(scalar input) {
    std::optional<scalar> premise;

    premise_pairs_.clear();

    for (size_t idx = 0; idx < membership_->getDiscourseSize(); idx++) {
        premise = membership_->calculate(input, membership_->getParamSet(idx));
        if (premise != std::nullopt && fabs(premise.value()) >= fc::eps) {
            premise_pairs_.emplace_back(std::make_pair(premise.value(), idx));
        }
    }
}

FuzzyLogic::FuzzyLogic() {
    e = std::make_unique<Fuzzification>();
    ec = std::make_unique<Fuzzification>();
    u = std::make_unique<Fuzzification>();
}

FuzzyLogic::~FuzzyLogic() {}

void FuzzyLogic::inference(void) {
    Inference_t inference_set;

    size_t rows = e->premise_pairs_.size();
    size_t cols = ec->premise_pairs_.size();

    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            const auto [e_premise, e_discourse_id] = e->premise_pairs_.at(i);
            const auto [ec_premise, ec_discourse_id] = ec->premise_pairs_.at(j);
            inference_set.weight = e_premise * ec_premise;
            inference_set.rule = static_cast<int>(rule_table_(e_discourse_id, ec_discourse_id));
            // If the rule overlaps the map stored, we need to keep the higher weight one.
            if(auto res = inference_map_.find(inference_set.rule); res != inference_map_.end()) {
                res->second = fmax(inference_set.weight, res->second);
            } else {
                inference_map_.insert({inference_set.rule, inference_set.weight});
            }
        }
    }
}


CentroidPair FuzzyLogic::centroid(size_t rule_id, scalar truncation_premise) {
    scalar x, y;
    scalar x_centroid = 0, area = 0;

    const auto params = u->membership_->getParamSet(rule_id);
    const auto [minimum, maximum] = u->membership_->getRange(rule_id);
    const scalar dx = (maximum - minimum) / resolution_;

    for (int j = 0; j < resolution_; j++) {
        x = minimum + (j + 0.5) * dx;
        y = fmin(u->membership_->calculate(x, params).value_or(0.0), truncation_premise);
        x_centroid += x * y;
        area += y;
    }

    return CentroidPair{x_centroid, area};
}


scalar FuzzyLogic::defuzzify(void) {
    // centroid method
    scalar num = 0, den = 0;
    for (const auto& [rule, weight]: inference_map_) {
        const size_t rule_id = static_cast<size_t>(rule) + u->membership_->getDiscourseSize() / 2;
        const auto [x_centroid, area] = centroid(rule_id, weight);
        num += x_centroid;
        den += area;
    }

    return num / den;
}

void FuzzyLogic::setFuzzyRules(const Matrix &rule_table) {
    if (static_cast<size_t>(rule_table.cols()) != ec->membership_->getDiscourseSize()
        || static_cast<size_t>(rule_table.rows()) != e->membership_->getDiscourseSize())
    {
        throw std::length_error("[fuzzy logic] fuzzy rules matrix size needs to fit the discourse size");
    }
    rule_table_ = rule_table;
}