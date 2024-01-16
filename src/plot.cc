#if FC_USE_MATPLOTLIB

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include "fuzzy/fuzzification.h"
#include "fuzzy/fuzzy_logic.h"
#include "fuzzy/membership.h"

void fc::Fuzzification::plotMembershipFunctions(bool show)
{
    int n = 500;
    std::vector<double> x(n);
    std::vector<double> y(n);
    std::vector<double> w(n, 0);

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }

    size_t discourse_size = membership()->getDiscourseSize();
    for (size_t id = 0; id < discourse_size; id++) {
        const auto params = membership()->getParamSet(id);
        const auto [minimum, maximum] = membership()->getRange(id);
        double dx = (maximum - minimum) / n;
        for (int i = 0; i < n; i++) {
            x.at(i) = minimum + i * dx;
            y.at(i) = membership()->calculate(x.at(i), params).value_or(0);
        }
        plt::plot(x, y,
                  {
                      {"linewidth", "2.0"}
        });
        plt::plot(x, w, "r");
    }
    plt::ylabel("Degree of membership");
    plt::xlabel("input/output");
    plt::xlim(-3.0, 3.0);

    if (show) {
        plt::show();
    }
    else {
        switch (membership()->getType()) {
        case MembershipType::Triangle:
            plt::title("Membership Functions: Triangle");
            plt::save("assets/" + getName() + "_membership_triangle.png");
            break;
        case MembershipType::Trapezoid:
            plt::title("Membership Functions: Trapezoid");
            plt::save("assets/" + getName() + "_membership_trapezoid.png");
            break;
        case MembershipType::Gaussian:
            plt::title("Membership Functions: Gaussian");
            plt::save("assets/" + getName() + "_membership_gaussian.png");
            break;
        case MembershipType::None:
            /* fall through */
        default:
            throw std::invalid_argument("[membership error] set membership type first");
        }
    }
    plt::close();
}

void fc::FuzzyLogic::plotFuzzyControlSurface(bool show)
{
    std::vector<std::vector<double>> x;
    std::vector<std::vector<double>> y;
    std::vector<std::vector<double>> z;
    double e_discourse_bound = static_cast<double>(e->membership()->getDiscourseSize() / 2);
    double ec_discourse_bound = static_cast<double>(ec->membership()->getDiscourseSize() / 2);

    for (double i = -e_discourse_bound; i <= e_discourse_bound; i++) {
        std::vector<double> x_row;
        std::vector<double> y_row;
        std::vector<double> z_row;
        for (double j = -ec_discourse_bound; j <= ec_discourse_bound; j++) {
            x_row.push_back(i);
            y_row.push_back(j);
            z_row.push_back(rule_table_(static_cast<size_t>(i + e_discourse_bound),
                                        static_cast<size_t>(j + ec_discourse_bound)));
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
    plt::plot_surface(x, y, z,
                      {
                          {  "linewidth",    "2"},
                          {"antialiased", "True"}
    });
    plt::xlabel("e");
    plt::ylabel("ec");
    plt::set_zlabel("u");
    plt::title("Fuzzy Control Surface");
    if (show) {
        plt::show();
    }
    else {
        plt::save("assets/fuzzy_control_surface.png");
    }
    plt::close();
}

void fc::FuzzyLogic::plotControl(const std::string &filename_prefix,
                                 const std::string &filename_suffix, bool show)
{
    size_t n = control_plot_.size();
    std::vector<double> x(n);
    std::vector<double> y(n);
    std::vector<double> w(n);

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
    plt::named_plot("Target", x, w, "r--");
    plt::ylabel("value");
    plt::xlabel("times");
    plt::xlim(static_cast<size_t>(0), n);
    plt::title("Fuzzy Control Demo: Target and Actual");
    plt::legend();
    if (show) {
        plt::show();
    }
    else {
        plt::save("assets/" + filename_prefix + "fc_demo_target_and_actual" + filename_suffix
                  + ".png");
    }
    plt::close();
}

void fc::FuzzyLogic::plotControlErr(const std::string &filename_prefix,
                                    const std::string &filename_suffix, bool show)
{
    size_t n = control_err_plot_.size();
    std::vector<double> x(n);
    std::vector<double> y1(n);
    std::vector<double> y2(n);
    std::vector<double> w(n, 0);

    for (size_t i = 0; i < n; i++) {
        x.at(i) = i;
        y1.at(i) = control_err_plot_.at(i).err;
        y2.at(i) = control_err_plot_.at(i).d_err;
    }

    // determine the figure config
    if (!show) {
        plt::figure_size(1280, 768);
    }

    plt::named_plot("Error", x, y1);
    plt::named_plot("DError", x, y2);
    plt::plot(x, w, "r--");
    plt::ylabel("value");
    plt::xlabel("times");
    plt::xlim(static_cast<size_t>(0), n);
    plt::title("Fuzzy Control Demo: Error and Derivative Error");
    plt::legend();
    if (show) {
        plt::show();
    }
    else {
        plt::save("assets/" + filename_prefix + "fc_demo_err_and_derr" + filename_suffix + ".png");
    }
    plt::close();
}

#endif
