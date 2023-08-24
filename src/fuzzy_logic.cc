#include "fuzzy/fuzzy_logic.h"
#include <iostream>
#include <cmath>

using namespace fc;

Fuzzification::Fuzzification(const std::string name, scalar bound)
    : factor_(0), bound_(bound), name_(name)
{
    membership_ = std::make_unique<Membership>(name);
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

const std::string& Fuzzification::getName(void) {
    return name_;
}


FuzzyLogic::FuzzyLogic()
    : control_(Err_t{0, 0, 0})
{
    e  = std::make_unique<Fuzzification>("e");
    ec = std::make_unique<Fuzzification>("ec");
    u  = std::make_unique<Fuzzification>("u");
}

FuzzyLogic::~FuzzyLogic() {}

scalar FuzzyLogic::algo(Control_t input) {
    scalar output;

    // calculate the basic control params and normalize them to discourse range
    control_.err   = e->factor_ * (input.target - input.actual);
    control_.d_err = ec->factor_ * (control_.err - control_.prev_err);
    control_.prev_err = control_.err;

    e->fuzzify(control_.err);
    ec->fuzzify(control_.d_err);

    inference();

    output = defuzzify() * u->factor_;

    if (output >= u->bound_) output = u->bound_;
    if (output <= -u->bound_) output = -u->bound_;

    return output;
}

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


#if FC_USE_MATPLOTLIB
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void Fuzzification::plotMembershipFunctions(void) {
    int n = 500;
    std::vector<double> x(n), y(n), w(n, 0);

    plt::figure_size(1280, 768);
    size_t discourse_size = membership_->getDiscourseSize();
    switch (membership_->getType()) {
        case membershipType::Triangle:
            for (size_t id = 0; id < discourse_size; id++) {
                const auto params = membership_->getParamSet(id);
                const auto [minimum, maximum] = membership_->getRange(id);
                double dx = (maximum - minimum) / n;
                for (int i = 0; i < n; i++) {
                    x.at(i) = minimum + i * dx;
                    y.at(i) = membership_->calculate(x.at(i), params).value_or(0);
                }
                plt::plot(x, y);
                plt::plot(x, w,"r--");
            }
            plt::ylabel("Degree of membership");
            plt::xlabel("input/output");
            plt::title("Membership Functions: Triangle");
            plt::save("assets/" + getName() + "_membership_triangle.png");
            break;
        case membershipType::Trapezoid:
            break;
        case membershipType::Gaussian:
            break;
        case membershipType::None:
            /* fall through */
        default:
            throw std::invalid_argument("[membership error] set membership type first");
    }
}

void FuzzyLogic::plotFuzzyControlSurface(void) {
    std::vector<std::vector<double>> x, y, z;
    double e_discourse_bound = static_cast<double>(e->membership_->getDiscourseSize() / 2);
    double ec_discourse_bound = static_cast<double>(ec->membership_->getDiscourseSize() / 2);

    printf("Rows(%f,%f) Cols(%f,%f)\n", -e_discourse_bound, e_discourse_bound, -ec_discourse_bound, ec_discourse_bound);

    for (double i = -e_discourse_bound; i <= e_discourse_bound;  i++) {
        std::vector<double> x_row, y_row, z_row;
        for (double j = -ec_discourse_bound; j <= ec_discourse_bound;  j++) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(rule_table_(static_cast<size_t>(i + e_discourse_bound), static_cast<size_t>(j + ec_discourse_bound)));
        }
        x.push_back(x_row);
        y.push_back(y_row);
        z.push_back(z_row);
    }

    // elevation=18, azimuth=-133
    long id = plt::figure();
    plt::plot_surface(x, y, z, {{"linewidth", "2"}, {"antialiased", "True"}}, id);
    plt::xlabel("e");
    plt::ylabel("ec");
    plt::set_zlabel("u");
    plt::title("Fuzzy Control Surface");
    plt::show();
    // plt::save("assets/fuzzy_control_surface.png");
}

#endif