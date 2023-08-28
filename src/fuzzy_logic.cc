#include "fuzzy/fuzzy_logic.h"
#include <iostream>
#include <cmath>

using namespace fc;

//? Fuzzification
Fuzzification::Fuzzification(const std::string name, scalar bound)
    : name_(name),
      state_(FuzzyProcess::FuzzyUninit),
      factor_(0),
      bound_(bound)
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
    factor_ = static_cast<scalar>(membership_->getDiscourseSize() / 2) / bound_;
    if (reverse) {
        factor_ = 1 / factor_;
    }

    state_ = FuzzyProcess::FuzzyInit;
}

void Fuzzification::fuzzify(scalar input) {
    std::optional<scalar> premise;

    premise_pairs_.clear();

    for (size_t idx = 0; idx < membership_->getDiscourseSize(); idx++) {
        premise = membership_->calculate(input, membership_->getParamSet(idx));
        // dbgmsgln("[fuzzify] %s - premise: %.3f, idx: %llu, input: %.3f", getName().c_str(), premise.value(), idx, input);
        if (premise != std::nullopt && fabs(premise.value()) >= fc::eps) {
            premise_pairs_.emplace_back(std::make_pair(premise.value(), idx));
            dbgmsgln("[fuzzify] %s - input: %.3f premise: %.3f, discourse ID: %llu", getName().c_str(), input, premise.value(), idx);
        }
    }
}

void Fuzzification::setFactor(const scalar ratio, const bool reverse) {
    if (state_ != FuzzyProcess::FuzzyInit) {
        dbgmsgln("[fuzzification] please init first");
        return;
    }

    scalar input_ratio = ratio > fc::eps ? ratio : 0.1;
    factor_ = input_ratio * static_cast<scalar>(membership_->getDiscourseSize() / 2) / bound_;
    if (reverse) {
        factor_ = 1 / factor_;
    }
}

void Fuzzification::setBound(scalar bound) {
    if (state_ != FuzzyProcess::FuzzyInit) {
        dbgmsgln("[fuzzification] please init first");
        return;
    }

    if (bound < -eps) {
        dbgmsgln("[fuzzification] 'bound' needs to be positive");
        return;
    }

    bound_ = bound;
}

const std::string& Fuzzification::getName(void) {
    return name_;
}

scalar Fuzzification::getFactor(void) {
    return factor_;
}

scalar Fuzzification::getBound(void) {
    return bound_;
}


//? FuzzyLogic
FuzzyLogic::FuzzyLogic(int resolution)
    : resolution_(resolution),
      control_(Err_t{0, 0, 0})
{
    e  = std::make_unique<Fuzzification>("e");
    ec = std::make_unique<Fuzzification>("ec");
    u  = std::make_unique<Fuzzification>("u");
}

FuzzyLogic::~FuzzyLogic() {}

scalar sign(scalar x) {
    if (x >= eps) {
        return 1.0;
    } else if (x <= -eps) {
        return -1.0;
    } else {
        return 0.0;
    }
}

void FuzzyLogic::rangeCheck(scalar& input, Membership* ptr) {
    if (input > ptr->getMaximum()) {
        input = ptr->getMaximum();
    }

    if (input < ptr->getMinimum()) {
        input = ptr->getMinimum();
    }
}

static size_t iter = 0;
scalar FuzzyLogic::algo(Control_t input, bool show_control_info) {
    scalar output, err, d_err;

    // TODO: input may out of max bound, acquiring limitation for scaling.
    // calculate the basic control params and normalize them to discourse range
    err   = input.target - input.actual;
    d_err = err - control_.prev_err;

    control_.err      = e->getFactor() * (fabs(err) > e->getBound() ? sign(err) * e->getBound() : err);
    control_.d_err    = ec->getFactor() * (fabs(d_err) > ec->getBound() ? sign(err) * ec->getBound() : d_err);
    control_.prev_err = fabs(err) > e->getBound() ? sign(err) * e->getBound() : err;

    // ensure the factor will not make e or ec out of range
    rangeCheck(control_.err, e->membership_.get());
    rangeCheck(control_.d_err, ec->membership_.get());

    e->fuzzify(control_.err);
    ec->fuzzify(control_.d_err);

    inference();

    output = defuzzify() * u->getFactor();

    if (output >= u->getBound()) output = u->getBound();
    if (output <= -u->getBound()) output = -u->getBound();

    if (show_control_info) {
        if (iter == 0) {
            dbgmsgln("Times    Target    Actual    Error    DError    PError");
        }
        dbgmsgln("%04llu     %06.2f    %06.2f    %06.3f   %06.3f    %06.3f",
            iter++, input.target, input.actual, control_.err, control_.d_err, control_.prev_err);
    }

#if FC_USE_MATPLOTLIB
    control_plot_.emplace_back(Control_t{input.target, input.actual});
    control_err_plot_.emplace_back(control_);
#endif

    return output;
}

void FuzzyLogic::inference(void) {
    Inference_t inference_set;

    size_t rows = e->premise_pairs_.size();
    size_t cols = ec->premise_pairs_.size();

    size_t e_discourse_size  = e->membership_->getDiscourseSize();
    size_t ec_discourse_size = ec->membership_->getDiscourseSize();

    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < cols; j++) {
            const auto [e_premise, e_discourse_id]   = e->premise_pairs_.at(i);
            const auto [ec_premise, ec_discourse_id] = ec->premise_pairs_.at(j);
            inference_set.weight = e_premise * ec_premise;
            inference_set.rule = static_cast<int>(rule_table_(e_discourse_size - 1 - e_discourse_id, ec_discourse_size - 1 - ec_discourse_id));
            dbgmsgln("[Inference] rule %llu => RuleTable(%llu, %llu)=%d, Weight=%.4f",
                (i * rows + j), e_discourse_id, ec_discourse_id, inference_set.rule, inference_set.weight);
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
        dbgmsgln("[defuzzify] rule id %d, Weight %.4f", rule, weight);
        const auto [x_centroid, area] = centroid(rule_id, weight);
        num += x_centroid;
        den += area;
    }
    // prepare for next iteration
    inference_map_.clear();

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


void FuzzyLogic::getInfo(void) {
    std::cout << std::endl;
    infomsgln("Fuzzy logic controller info:");
    infomsgln("=> discourse e:  [%.3f, %.3f], min-max[%.2f, %.2f]",
        -e->getBound(), e->getBound(), e->membership_->getMinimum(), e->membership_->getMaximum());
    infomsgln("=> discourse ec: [%.3f, %.3f], min-max[%.2f, %.2f]",
        -ec->getBound(), ec->getBound(), e->membership_->getMinimum(), e->membership_->getMaximum());
    infomsgln("=> discourse u:  [%.3f, %.3f], min-max[%.2f, %.2f]",
        -u->getBound(), u->getBound(), e->membership_->getMinimum(), e->membership_->getMaximum());
    infomsgln("=> error quantifying factor [Ke]:             %.4f", e->getFactor());
    infomsgln("=> derivative error quantifying factor [Kec]: %.4f", ec->getFactor());
    infomsgln("=> output scaling factor [Ku]:                %.4f", u->getFactor());
    std::cout << std::endl;
}

#if FC_USE_MATPLOTLIB

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void Fuzzification::plotMembershipFunctions(bool show) {
    int n = 500;
    std::vector<double> x(n), y(n), w(n, 0);

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }

    size_t discourse_size = membership_->getDiscourseSize();
    for (size_t id = 0; id < discourse_size; id++) {
        const auto params = membership_->getParamSet(id);
        const auto [minimum, maximum] = membership_->getRange(id);
        double dx = (maximum - minimum) / n;
        for (int i = 0; i < n; i++) {
            x.at(i) = minimum + i * dx;
            y.at(i) = membership_->calculate(x.at(i), params).value_or(0);
        }
        plt::plot(x, y, {{"linewidth", "2.0"}});
        plt::plot(x, w,"r");
    }
    plt::ylabel("Degree of membership");
    plt::xlabel("input/output");
    plt::xlim(-3.0, 3.0);

    if (show) {
        plt::show();
    } else {
        switch (membership_->getType()) {
            case membershipType::Triangle:
                plt::title("Membership Functions: Triangle");
                plt::save("assets/" + getName() + "_membership_triangle.png");
                break;
            case membershipType::Trapezoid:
                plt::title("Membership Functions: Trapezoid");
                plt::save("assets/" + getName() + "_membership_trapezoid.png");
                break;
            case membershipType::Gaussian:
                plt::title("Membership Functions: Gaussian");
                plt::save("assets/" + getName() + "_membership_gaussian.png");
                break;
            case membershipType::None:
                /* fall through */
            default:
                throw std::invalid_argument("[membership error] set membership type first");
        }
    }
    plt::close();
}

void FuzzyLogic::plotFuzzyControlSurface(bool show) {
    std::vector<std::vector<double>> x, y, z;
    double e_discourse_bound = static_cast<double>(e->membership_->getDiscourseSize() / 2);
    double ec_discourse_bound = static_cast<double>(ec->membership_->getDiscourseSize() / 2);

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

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }
    // elevation=20, azimuth=45
    plt::plot_surface(x, y, z, {{"linewidth", "2"}, {"antialiased", "True"}});
    plt::xlabel("e");
    plt::ylabel("ec");
    plt::set_zlabel("u");
    plt::title("Fuzzy Control Surface");
    if (show) {
        plt::show();
    } else {
        plt::save("assets/fuzzy_control_surface.png");
    }
    plt::close();
}

void FuzzyLogic::plotControl(std::string filename_suffix, bool show) {
    size_t n = control_plot_.size();
    std::vector<double> x(n), y(n), w(n);

    for (size_t i = 0; i < n; i++) {
        x.at(i) = i;
        y.at(i) = control_plot_.at(i).actual;
        w.at(i) = control_plot_.at(i).target;
    }

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }
    plt::named_plot("Actual", x, y);
    plt::named_plot("Target", x, w,"r--");
    plt::ylabel("value");
    plt::xlabel("times");
    plt::xlim((size_t)0, n);
    plt::title("Fuzzy Control Demo: Target and Actual");
    plt::legend();
    if (show) {
        plt::show();
    } else {
        plt::save("assets/fc_demo_target_and_actual" + filename_suffix + ".png");
    }
    plt::close();
}


void FuzzyLogic::plotControlErr(std::string filename_suffix, bool show) {
    size_t n = control_err_plot_.size();
    std::vector<double> x(n), y1(n), y2(n), w(n, 0);

    for (size_t i = 0; i < n; i++) {
        x.at(i)  = i;
        y1.at(i) = control_err_plot_.at(i).err;
        y2.at(i) = control_err_plot_.at(i).d_err;
    }

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }

    plt::named_plot("Error", x, y1);
    plt::named_plot("DError", x, y2);
    plt::plot(x, w,"r--");
    plt::ylabel("value");
    plt::xlabel("times");
    plt::xlim((size_t)0, n);
    plt::title("Fuzzy Control Demo: Error and Derivative Error");
    plt::legend();
    if (show) {
        plt::show();
    } else {
        plt::save("assets/fc_demo_err_and_derr" + filename_suffix + ".png");
    }
    plt::close();
}

#endif