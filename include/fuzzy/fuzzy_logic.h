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

#include "utils/base.h"
#include "utils/operation.h"
#include "fuzzy/membership.h"
#include "fuzzy/pid.h"
#include "Eigen/Eigen"

#include <unordered_map>
#include <memory>

namespace fc {

typedef std::pair<scalar/*premise*/, size_t/*discourse id*/> PremisePair;
typedef std::pair<scalar/* area */, scalar/*centroid*/> CentroidPair;

typedef struct Inference {
    scalar weight;
    int rule;
} Inference_t;

enum class FuzzyProcess {
    FuzzyUninit,
    FuzzyInit,
};

//? Note: e/ec factor=discourse_size/bound, u factor=bound/discourse_size
class Fuzzification {
    public:
        Fuzzification(const std::string name = "", scalar bound = 0.0);
        ~Fuzzification();

        void init(const scalar bound,
                  const bool reverse,
                  const membershipType type,
                  const scalar input_params[],
                  const uint8_t params_num);
        void fuzzify(scalar input);

        void setFactor(const scalar ratio, const bool reverse);
        void setBound(const scalar bound);

        const std::string& getName(void);
        scalar getFactor(void);
        scalar getBound(void);

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


typedef struct Err {
    scalar err;
    scalar prev_err;
    scalar d_err;
} Err_t;

typedef struct Control {
    scalar target;
    scalar actual;
} Control_t;

class FuzzyLogic {
    public:
        FuzzyLogic(int resolution = 200);
        ~FuzzyLogic();

        scalar algo(const Control_t input, bool use_p_ctrl = false, const scalar output_exp_scale = 0.0);
        void setFuzzyRules(const Matrix &rule_table);
        void getInfo(void);

#if FC_USE_MATPLOTLIB
        void plotFuzzyControlSurface(bool show = false);
        void plotControl(std::string filename_prefix = "", std::string filename_suffix = "", bool show = false);
        void plotControlErr(std::string filename_prefix = "", std::string filename_suffix = "", bool show = false);

        std::vector<Control_t> control_plot_;
        std::vector<Err_t> control_err_plot_;
#endif

        std::unique_ptr<Fuzzification> e;
        std::unique_ptr<Fuzzification> ec;
        std::unique_ptr<Fuzzification> u;

        std::unique_ptr<PController> p_ctrl_;
    private:
        void inference(void);
        scalar defuzzify(void);
        CentroidPair centroid(size_t rule_id, scalar truncation_premise);
        void rangeCheck(scalar& input, Membership* ptr);
        bool controllerSwitchCheck(void);

        int resolution_;
        Matrix rule_table_;
        std::unordered_map<int, scalar>inference_map_;

        Err_t control_;
};

}

#endif //! __FC_FUZZY_CONTROL__H__