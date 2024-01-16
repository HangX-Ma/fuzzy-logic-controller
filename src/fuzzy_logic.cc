#include "fuzzy/fuzzy_logic.h"
#include "fuzzy/membership.h"
#include "fuzzy/fuzzification.h"

#include <spdlog/spdlog.h>

#include <iostream>
#include <cmath>

namespace fc
{

//? FuzzyLogic
FuzzyLogic::FuzzyLogic(int resolution) : resolution_(resolution), control_(Err_t{0, 0, 0})
{
    e = std::make_unique<Fuzzification>("e");
    ec = std::make_unique<Fuzzification>("ec");
    u = std::make_unique<Fuzzification>("u");

    p_ctrl = std::make_unique<PController>();
}

FuzzyLogic::~FuzzyLogic() {}

scalar sign(scalar x)
{
    if (x >= eps) {
        return 1.0;
    }
    if (x <= -eps) {
        return -1.0;
    }
    return 0.0;
}

void FuzzyLogic::rangeCheck(scalar &input, Membership *membership)
{
    if (input > membership->getMaximum()) {
        input = membership->getMaximum();
    }

    if (input < membership->getMinimum()) {
        input = membership->getMinimum();
    }
}

bool FuzzyLogic::controllerSwitchCheck(scalar err, scalar d_err)
{
    if (err > e->getBound() * switch_ratio_ || err < -e->getBound() * switch_ratio_) {
        return true;
    }
    if (d_err > ec->getBound() * switch_ratio_ || d_err < -ec->getBound() * switch_ratio_) {
        return true;
    }

    return false;
}

static size_t iter = 0;
scalar FuzzyLogic::algo(const Control_t input, bool use_p_ctrl, const scalar output_exp_scale)
{
    scalar output;
    scalar defuzzify_output;
    scalar err;
    scalar d_err;

    // DONE(HangX-Ma): input may out of max bound, acquiring limitation for scaling.
    // calculate the basic control params and normalize them to discourse range
    err = input.target - input.actual;
    d_err = err - control_.prev_err;

    control_.err = e->getFactor() * (fabs(err) > e->getBound() ? sign(err) * e->getBound() : err);
    control_.d_err
        = ec->getFactor() * (fabs(d_err) > ec->getBound() ? sign(err) * ec->getBound() : d_err);
    control_.prev_err = fabs(err) > e->getBound() ? sign(err) * e->getBound() : err;

    if (use_p_ctrl && controllerSwitchCheck(err, d_err)) {
        output = p_ctrl->algo(err);
        if (output >= u->getBound() * proportional_u_ratio_) {
            output = u->getBound() * proportional_u_ratio_;
        }
        if (output <= -u->getBound() * proportional_u_ratio_) {
            output = -u->getBound() * proportional_u_ratio_;
        }

        spdlog::debug("[Proportional Ctrl]: {:^8}{:^8}{:^8}{:^8}", "Times", "Target", "Actual",
                      "Error");
        spdlog::debug("[Proportional Ctrl]: {:^8d}{:^8.3f}{:^8.3f}{:^8.3f}\n", iter, input.target,
                      input.actual, err);
    }
    else {
        // ensure the factor will not make e or ec out of range
        rangeCheck(control_.err, e->membership().get());
        rangeCheck(control_.d_err, ec->membership().get());

        e->fuzzify(control_.err);
        ec->fuzzify(control_.d_err);

        inference();

        defuzzify_output = defuzzify();

        // make output bigger when error is big: (sign) (fabs(output))^(exp(scale)) * Ku
        if ((fabs(output_exp_scale) > fc::eps && fabs(output_exp_scale) < 1.0)
            && fabs(defuzzify_output) > 0.5)
        {
            output = sign(defuzzify_output) * pow(fabs(defuzzify_output), exp(output_exp_scale))
                     * u->getFactor();
        }
        else {
            output = defuzzify_output * u->getFactor();
        }

        if (output >= u->getBound()) {
            output = u->getBound();
        }
        if (output <= -u->getBound()) {
            output = -u->getBound();
        }
        // control info
        spdlog::debug("[Fuzzy Ctrl]: {:^8}{:^8}{:^8}{:^8}{:^8}{:^8}", "Times", "Target", "Actual",
                      "Error", "DError", "PError");
        spdlog::debug("[Fuzzy Ctrl]: {:^8d}{:^8.3f}{:^8.3f}{:^8.3f}{:^8.3f}{:^8.3f}\n", iter,
                      input.target, input.actual, control_.err, control_.d_err, control_.prev_err);
    }
    iter += 1;

#if FC_USE_MATPLOTLIB
    control_plot_.emplace_back(Control_t{input.target, input.actual});
    control_err_plot_.emplace_back(Err_t{err, -1 /*unused*/, d_err});
#endif

    return output;
}

void FuzzyLogic::inference()
{
    Inference_t inference_set;

    size_t rows = e->premiseVec().size();
    size_t cols = ec->premiseVec().size();

    size_t e_discourse_size = e->membership()->getDiscourseSize();
    size_t ec_discourse_size = ec->membership()->getDiscourseSize();

    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            const auto [e_premise, e_discourse_id] = e->premiseVec().at(i);
            const auto [ec_premise, ec_discourse_id] = ec->premiseVec().at(j);
            inference_set.weight = e_premise * ec_premise;
            inference_set.rule = static_cast<int>(rule_table_(
                e_discourse_size - 1 - e_discourse_id, ec_discourse_size - 1 - ec_discourse_id));
            spdlog::debug("[Inference] rule {:<2d} => RuleTable({}, {}): {: d}, Weight: {:< 5.3f}",
                          (i * rows + j), e_discourse_id, ec_discourse_id, inference_set.rule,
                          inference_set.weight);
            // If the rule overlaps the map stored, we need to keep the higher weight one.
            if (auto res = inference_map_.find(inference_set.rule); res != inference_map_.end()) {
                res->second = fmax(inference_set.weight, res->second);
            }
            else {
                inference_map_.insert({inference_set.rule, inference_set.weight});
            }
        }
    }
}

CentroidPair FuzzyLogic::centroid(size_t rule_id, scalar truncation_premise)
{
    scalar x;
    scalar y;
    scalar x_centroid = 0;
    scalar area = 0;

    const auto params = u->membership()->getParamSet(rule_id);
    const auto [minimum, maximum] = u->membership()->getRange(rule_id);
    const scalar dx = (maximum - minimum) / resolution_;

    for (int j = 0; j < resolution_; j++) {
        x = minimum + (j + 0.5) * dx;
        y = fmin(u->membership()->calculate(x, params).value_or(0.0), truncation_premise);
        x_centroid += x * y;
        area += y;
    }

    return CentroidPair{x_centroid, area};
}

scalar FuzzyLogic::defuzzify()
{
    // centroid method
    scalar num = 0;
    scalar den = 0;
    for (const auto &[rule, weight] : inference_map_) {
        const size_t rule_id = static_cast<size_t>(rule) + u->membership()->getDiscourseSize() / 2;
        spdlog::debug("[Defuzzify] rule id {:< 2d}, Weight {:.3f}", rule, weight);
        const auto [x_centroid, area] = centroid(rule_id, weight);
        num += x_centroid;
        den += area;
    }
    // prepare for next iteration
    inference_map_.clear();

    return num / den;
}

void FuzzyLogic::setFuzzyRules(const Matrix &rule_table)
{
    if (static_cast<size_t>(rule_table.cols()) != ec->membership()->getDiscourseSize()
        || static_cast<size_t>(rule_table.rows()) != e->membership()->getDiscourseSize())
    {
        throw std::length_error(
            "[fuzzy logic] fuzzy rules matrix size needs to fit the discourse size");
    }
    rule_table_ = rule_table;
}

void FuzzyLogic::getInfo()
{
    std::cout << std::endl;
    spdlog::info("Fuzzy logic controller info:");
    spdlog::info("=> discourse  e: [{:<5.3f}, {:<5.3f}], min-max[{:<4.2f}, {:<4.2f}]",
                 -e->getBound(), e->getBound(), e->membership()->getMinimum(),
                 e->membership()->getMaximum());
    spdlog::info("=> discourse ec: [{:<5.3f}, {:<5.3f}], min-max[{:<4.2f}, {:<4.2f}]",
                 -ec->getBound(), ec->getBound(), e->membership()->getMinimum(),
                 e->membership()->getMaximum());
    spdlog::info("=> discourse  u: [{:<5.3f}, {:<5.3f}], min-max[{:<4.2f}, {:<4.2f}]",
                 -u->getBound(), u->getBound(), e->membership()->getMinimum(),
                 e->membership()->getMaximum());
    spdlog::info("=> error quantifying factor             [Ke]: {:.4f}", e->getFactor());
    spdlog::info("=> derivative error quantifying factor [Kec]: {:.4f}", ec->getFactor());
    spdlog::info("=> output scaling factor                [Ku]: {:.4f}", u->getFactor());
    spdlog::info("=> proportional controller              [Kp]: {:.4f}", p_ctrl->getProportional());
    std::cout << std::endl;
}
} // namespace fc