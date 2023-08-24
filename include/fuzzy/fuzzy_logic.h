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

#include "base.h"
#include "operation.h"
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

        void setFactor(scalar factor);
        scalar getFactor(void);

        std::unique_ptr<Membership> membership_;
        std::vector<PremisePair> premise_pairs_;
    private:
        scalar factor_; // quantifying/scaling factor
        scalar bound_;  // upper bound
};

class FuzzyLogic {
    public:
        FuzzyLogic();
        ~FuzzyLogic();

        void inference(void);
        scalar defuzzify(void);
        void setFuzzyRules(const Matrix &rule_table);

        Matrix rule_table_;
        std::unique_ptr<Fuzzification> e;
        std::unique_ptr<Fuzzification> ec;
        std::unique_ptr<Fuzzification> u;

    private:
        CentroidPair centroid(size_t rule_id, scalar truncation_premise);
        int resolution_;
        std::unordered_map<int, scalar>inference_map_;
};

}

#endif //! __FC_FUZZY_CONTROL__H__