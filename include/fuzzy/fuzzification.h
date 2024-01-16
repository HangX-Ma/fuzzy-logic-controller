/**
 * @file fuzzification.h
 * @author HangX-Ma (contour.9x@gmail.com)
 * @brief Fuzzification
 * @version 0.1
 * @date 2024-01-16
 */
#ifndef FC_FUZZIFICATION_H
#define FC_FUZZIFICATION_H

#include "fuzzy/utils.h"
#include <memory>
#include <utility>

namespace fc
{

//? Note: e/ec factor=discourse_size/bound, u factor=bound/discourse_size

class Membership;
enum class MembershipType : uint8_t;

class Fuzzification
{
public:
    explicit Fuzzification(const std::string &name = "", scalar bound = 0.0);
    ~Fuzzification() = default;

    void init(scalar bound, bool reverse, MembershipType type, const scalar input_params[],
              uint8_t params_num);
    void fuzzify(scalar input);

    // setter
    void setFactor(scalar ratio, bool reverse);
    void setBound(scalar bound);

    // getter
    auto membership() { return membership_; }
    const std::vector<PremisePair> &premiseVec() const { return premise_pairs_; }
    const std::string &getName() const;
    scalar getFactor() const;
    scalar getBound() const;

#if FC_USE_MATPLOTLIB
    void plotMembershipFunctions(bool show = false);
#endif

protected:
    enum class FuzzyProcess
    {
        FuzzyUninit,
        FuzzyInit,
    };
    std::string name;
    FuzzyProcess state{FuzzyProcess::FuzzyUninit};

private:
    std::shared_ptr<Membership> membership_{nullptr};
    std::vector<PremisePair> premise_pairs_;

    scalar factor_; // quantifying/scaling factor
    scalar bound_;  // upper bound
};

using Inference_t = struct Inference
{
    scalar weight;
    int rule;
};

} // namespace fc

#endif