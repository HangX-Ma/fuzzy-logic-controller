/**
 * @file fuzzy_logic.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzy logic
 * @version 0.3
 * @date 2022-07-15
 * @date 2023-08-22
 * @date 2024-01-16
 */

#ifndef FC_FUZZY_LOGIC_H
#define FC_FUZZY_LOGIC_H

#include "fuzzy/utils.h"
#include "fuzzy/pid.h"

#include <spdlog/spdlog.h>
#include <unordered_map>
#include <memory>

namespace fc
{

using Err_t = struct Err
{
    scalar err;
    scalar prev_err;
    scalar d_err;
};

using Control_t = struct Control
{
    scalar target;
    scalar actual;
};

class Fuzzification;
class Membership;

class FuzzyLogic
{
public:
    explicit FuzzyLogic(int resolution = 200);
    ~FuzzyLogic();

    scalar algo(Control_t input, bool use_p_ctrl = false, scalar output_exp_scale = 0.0);
    void setFuzzyRules(const Matrix &rule_table);
    void setSwitchRatio(double switch_ratio) { switch_ratio_ = switch_ratio; }
    void setProportionalURatio(double ratio) { proportional_u_ratio_ = ratio; }
    void getInfo();

#if FC_USE_MATPLOTLIB
    void plotFuzzyControlSurface(bool show = false);
    void plotControl(const std::string &filename_prefix = "",
                     const std::string &filename_suffix = "", bool show = false);
    void plotControlErr(const std::string &filename_prefix = "",
                        const std::string &filename_suffix = "", bool show = false);
#endif

    std::unique_ptr<Fuzzification> e;
    std::unique_ptr<Fuzzification> ec;
    std::unique_ptr<Fuzzification> u;

    std::unique_ptr<PController> p_ctrl;

private:
    void inference();
    scalar defuzzify();
    CentroidPair centroid(size_t rule_id, scalar truncation_premise);

    void rangeCheck(scalar &input, Membership *membership);
    bool controllerSwitchCheck(scalar err);

private:
    std::vector<Control_t> control_plot_;
    std::vector<Err_t> control_err_plot_;

private:
    int resolution_;
    Matrix rule_table_;
    std::unordered_map<int, scalar> inference_map_;

    Err_t control_;
    double switch_ratio_{1.0};
    double proportional_u_ratio_{1.0};
};

} // namespace fc

#endif