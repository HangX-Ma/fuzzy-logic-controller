/**
 * @file fuzzy_logic.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzy logic
 * @version 0.2
 * @date 2022-07-15
 * @date 2023-08-22
 */

#ifndef __FC_FUZZY_CONTROL__H__
#define __FC_FUZZY_CONTROL__H__

#include "fuzzy/base.h"
#include "fuzzy/operation.h"
#include "fuzzy/membership.h"
#include "fuzzy/pid.h"
#include "Eigen/Eigen"

#include <unordered_map>
#include <memory>

namespace fc
{

using PremisePair = std::pair<scalar, size_t>;
using CentroidPair = std::pair<scalar, scalar>;

using Inference_t = struct Inference
{
    scalar weight;
    int rule;
};

enum class FuzzyProcess
{
    FuzzyUninit,
    FuzzyInit,
};

//? Note: e/ec factor=discourse_size/bound, u factor=bound/discourse_size
class Fuzzification
{
public:
    explicit Fuzzification(std::string name = "", scalar bound = 0.0);
    ~Fuzzification();

    void init(scalar bound, bool reverse, MembershipType type, const scalar input_params[],
              uint8_t params_num);
    void fuzzify(scalar input);

    void setFactor(scalar ratio, bool reverse);
    void setBound(scalar bound);

    const std::string &getName();
    scalar getFactor();
    scalar getBound();

#if FC_USE_MATPLOTLIB
    void plotMembershipFunctions(bool show = false);
#endif

    std::unique_ptr<Membership> membership_;
    std::vector<PremisePair> premise_pairs_;

protected:
    std::string name_;
    FuzzyProcess state_;

private:
    scalar factor_; // quantifying/scaling factor
    scalar bound_;  // upper bound
};

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

class FuzzyLogic
{
public:
    explicit FuzzyLogic(int resolution = 200);
    ~FuzzyLogic();

    scalar algo(Control_t input, bool use_p_ctrl = false, scalar output_exp_scale = 0.0);
    void setFuzzyRules(const Matrix &rule_table);
    void setSwitchRatio(double switch_ratio) { switch_ratio_ = switch_ratio; }
    void getInfo();

#if FC_USE_MATPLOTLIB
    void plotFuzzyControlSurface(bool show = false);
    void plotControl(const std::string &filename_prefix = "", const std::string &filename_suffix = "",
                     bool show = false);
    void plotControlErr(const std::string &filename_prefix = "", const std::string &filename_suffix = "",
                        bool show = false);

    std::vector<Control_t> control_plot_;
    std::vector<Err_t> control_err_plot_;
#endif

    std::unique_ptr<Fuzzification> e;
    std::unique_ptr<Fuzzification> ec;
    std::unique_ptr<Fuzzification> u;

    std::unique_ptr<PController> p_ctrl_;

private:
    void inference();
    scalar defuzzify();
    CentroidPair centroid(size_t rule_id, scalar truncation_premise);
    void rangeCheck(scalar &input, Membership *ptr);
    bool controllerSwitchCheck();

    int resolution_;
    Matrix rule_table_;
    std::unordered_map<int, scalar> inference_map_;

    Err_t control_;
    double switch_ratio_{0};
};

} // namespace fc

#endif //! __FC_FUZZY_CONTROL__H__