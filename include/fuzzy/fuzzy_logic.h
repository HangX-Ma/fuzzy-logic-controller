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

//? Note: e/ec factor=discourse_size/bound, u factor=bound/discourse_size
class Fuzzification {
    public:
        Fuzzification(scalar bound = 0.0);
        ~Fuzzification();

        void init(const scalar bound,
                  const bool reverse,
                  const membershipType type,
                  const scalar input_params[],
                  const uint8_t params_num);
        void fuzzify(scalar input);

        std::unique_ptr<Membership> membership_;
        std::vector<PremisePair> premise_pairs_;

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
        FuzzyLogic();
        ~FuzzyLogic();

        scalar algo(Control_t input);
        void setFuzzyRules(const Matrix &rule_table);

#if FC_USE_MATPLOTLIB
        void plotFuzzyControlSurface(void);
#endif

        std::unique_ptr<Fuzzification> e;
        std::unique_ptr<Fuzzification> ec;
        std::unique_ptr<Fuzzification> u;

    private:
        void inference(void);
        scalar defuzzify(void);
        CentroidPair centroid(size_t rule_id, scalar truncation_premise);

        int resolution_;
        Matrix rule_table_;
        std::unordered_map<int, scalar>inference_map_;

        Err_t control_;
};

}

#endif //! __FC_FUZZY_CONTROL__H__