//
// Created by yangt on 18-12-21.
//
#include "non_constrained_optimiztion/line_search_step_length.hpp"
#include "non_constrained_optimiztion/polynomial.hpp"

namespace ncopt {
StepLengthFunction::StepLengthFunction(GradientProblem *problem)
        : problem_(problem),
          position_(problem->NumParameters()),
          direction_(problem->NumParameters()) {
}

void StepLengthFunction::Evaluate(double step,
                                  bool is_evaluate_gradient,
                                  Samples *output) {
    output->a = step;
    output->vector_x = position_ + step * direction_;

    if (!is_evaluate_gradient) {
        problem_->Evaluate(output->vector_x.data(), &(output->value), NULL);
        output->is_gradient_valid = false;
    } else {
        output->vector_gradient.resize(position_.size());
        problem_->Evaluate(output->vector_x.data(),
                           &(output->value),
                           output->vector_gradient.data());
        output->gradient = output->vector_gradient.dot(direction_);
        output->is_gradient_valid = true;
    }
    output->is_value_valid = true;
}

LineSearchStepLength::LineSearchStepLength(const LineSearchOption &option,
                                           StepLengthFunction *function)
        : options_(option), function_(function) {}

LineSearchStepLength *LineSearchStepLength::
Create(const LineSearchOption &options, StepLengthFunction *function) {
    switch (options.line_search_type) {
        case ARMIJO:return new ArimjoSearch(options, function);
        case WOLFE:return new WolfSearch(options, function);
    }
}

void LineSearchStepLength::FindPolynomialRoots(
        const Vector &polynomial, Vector *roots) const {
    int degree = polynomial.size() - 1;
//    CHECK_EQ(degree - 1, roots->size())
//        << "polynomial degree " << degree
//        << " is not coincide with roots degree " << roots->size();
    if (degree == 2) {
        roots->resize(1);
        if (fabs(polynomial(0)) < 1e-6) {
            (*roots)(0) = std::numeric_limits<double>::infinity();
        } else {
            (*roots)(0) = -polynomial(1) / 2.0 / polynomial(0);
        }
    }
    if (degree == 3) {
        if (fabs(polynomial(0)) < 1e-6 && fabs(polynomial(1)) < 1e-6) {
            roots->resize(1);
            (*roots)(0) = std::numeric_limits<double>::infinity();
        } else if (fabs(polynomial(0)) < 1e-6) {
            roots->resize(1);
            (*roots)(0) = -polynomial(1) / 2.0 / polynomial(0);
        } else {
            double judge =
                    pow(polynomial(1), 2) - 3.0 * polynomial(0) * polynomial(2);
            if (judge <= 0) {
                roots->resize(1);
                (*roots)(0) = -polynomial(1) / 3.0 / polynomial(0);
            } else {
                roots->resize(2);
                (*roots)(0) =
                        (-polynomial(1) + sqrt(judge)) / 3.0 / polynomial(0);
                (*roots)(1) =
                        (-polynomial(1) - sqrt(judge)) / 3.0 / polynomial(0);
            }
        }
    }
}
Vector LineSearchStepLength::PolynomialInterpolating(
        const std::vector<Samples> &samples) const {
    int degree = 0;
    if (options().interpolation_type == CUBIC) {
        degree = 4;
    } else if (options().interpolation_type == QUADRATIC) {
        degree = 3;
    }
    Matrix M(degree, degree);
    Vector rhs(degree);
    int row = 0;
    for (int i(0); i < samples.size(); ++i) {
        const auto &sample = samples.at(i);
        if (sample.is_value_valid) {
            for (int j(0); j < degree; ++j) {
                M(row, j) = pow(sample.a, degree - 1 - j);
            }
            rhs(row) = sample.value;
            row++;
            if (row == degree) {
                break;
            }
        }
        if (sample.is_gradient_valid) {
            for (int j(0); j < degree - 1; ++j) {
                M(row, j) = (degree - 1 - j) * pow(sample.a, degree - 2 - j);
            }
            rhs(row) = sample.gradient;
            row++;
            if (row == degree) {
                break;
            }
        }
    }
//    return M.lu().solve(rhs);
    Eigen::FullPivLU<Matrix> lu(M);
    return lu.setThreshold(0.0).solve(rhs);
}

double LineSearchStepLength::InterpolateMinimizingStepLength(
        const Samples &sample0,
        const Samples &sample1,
        const Samples &sample2,
        const double lower_step,
        const double upper_step) {
    if (options().interpolation_type == BISECTION) {
        return (sample0.a + sample1.a) / 2.0;
    }
    std::vector<Samples> samples;
    if (sample0.is_value_valid) {
        samples.push_back(sample0);
    }
    if (sample1.is_value_valid) {
        samples.push_back(sample1);
    }
    if (options().interpolation_type == QUADRATIC) {
        samples.push_back(Samples(sample2.a, sample2.value));
    } else if (options().interpolation_type == CUBIC) {
        samples.push_back(sample2);
    }

//    double step_optimal;
//    double optimal_value;
//    MinimizeInterpolatingPolynomial(samples,
//                                    lower_step,
//                                    upper_step,
//                                    &step_optimal,
//                                    &optimal_value);

    const Vector polynomial = PolynomialInterpolating(samples);

    if (polynomial.size() < 3) {
        LOG(ERROR) << "polynomial size is " << polynomial.size()
                   << ", which less than specified degree";
    }
    const double delta = (upper_step - lower_step) / 20.0;
    const double lower = lower_step + delta;
    const double upper = upper_step - delta;
    const double lower_value = EvaluatePolynomial(polynomial, lower);
    const double upper_value = EvaluatePolynomial(polynomial, upper);
    double step_optimal = (lower_step + upper_step) / 2.0;
    double step_optimal_valuel = EvaluatePolynomial(polynomial, step_optimal);

    if (step_optimal_valuel > lower_value) {
        step_optimal_valuel = lower_value;
        step_optimal = lower;
    }
    if (step_optimal_valuel > upper_value) {
        step_optimal_valuel = upper_value;
        step_optimal = upper;
    }
    Vector roots;
    FindPolynomialRoots(polynomial, &roots);
    for (int i(0); i < roots.size(); ++i) {
        if (roots(i) < lower_step || roots(i) > upper_step) {
            continue;
        }
        const double value = EvaluatePolynomial(polynomial, roots(i));
        if (value < step_optimal_valuel) {
            step_optimal_valuel = value;
            step_optimal = roots(i);
        }
    }
#ifdef DEBUG
    printf(">>>> lower:(%f, %f), upper:(%f, %f), optimal:(%f, %f)\n",
           lower,
           lower_value,
           upper,
           upper_value,
           step_optimal,
           step_optimal_valuel);
    plot_.SetApproxFunc(polynomial);
    plot_.plot();
#endif
    return step_optimal;
}

ArimjoSearch::ArimjoSearch(const LineSearchOption &option,
                           StepLengthFunction *function)
        : LineSearchStepLength(option, function) {

}

bool ArimjoSearch::DoSearch(const State &initial_state,
                            Summary *summary) {
    // set some parameters of inputing initial state
    summary->line_search_iteration_count = 0;
    StepLengthFunction::Samples current, previous;
    bool is_evaluate_gradient = false;
    if (options().interpolation_type == CUBIC) {
        is_evaluate_gradient = true;
    }
    function()->Evaluate(summary->initial_step, is_evaluate_gradient, &current);
    const Samples unused_sample;
    while (true) {
        // sufficient decrease condition:
        if (current.value < initial_state.cost + options().sufficient_decrease
                * current.a * initial_state.directional_derivative) {
            break;
        }
        // step length searching limit:
        if (current.a < options().min_line_search_step_length) {
            LOG(WARNING) << "step length " << current.a
                         << " is less than setting threshold "
                         << options().min_line_search_step_length;
            return false;
        }
        // choose optimal step length by funciton approximation
        double new_step = InterpolateMinimizingStepLength(
                unused_sample,
                previous,
                current,
                current.a * options().max_step_decrease_rate,
                current.a * options().min_step_decrease_rate);

        // update samples
        previous = current;
        function()->Evaluate(new_step, is_evaluate_gradient, &current);
        summary->line_search_iteration_count++;
    }
    summary->step = current.a;
    return true;
}

WolfSearch::WolfSearch(const LineSearchOption &option,
                       StepLengthFunction *function)
        : LineSearchStepLength(option, function) {

}

bool WolfSearch::Zoom(const State &initial_state,
                      Samples *s_lo,
                      Samples *s_hi,
                      double *step,
                      Summary *summary) {
    // interpolate between al and ah, than find min ai
    const double &cost0 = initial_state.cost;
    const double &dird0 = initial_state.directional_derivative;
    const double &c1 = options().sufficient_decrease;
    const double &c2 = options().sufficient_curvature_decrease;
#ifdef DEBUG
    printf(">>>> initial zoom section [%f, %f]\n", s_lo->a, s_hi->a);
#endif
    double weak_step = -1.0;
    Samples current;
    const Samples unused_sample;
    while (true) {
        const double lower_step = std::min(s_lo->a,
                                           s_hi->a);
        const double upper_step = std::max(s_lo->a,
                                           s_hi->a);
        // choose optimal step length by approximation
        *step =
                InterpolateMinimizingStepLength(unused_sample, *s_lo, *s_hi,
                                                lower_step, upper_step);
        function()->Evaluate(*step, true, &current);
        if (summary->line_search_iteration_count > 10) {
            if (weak_step > 0) {
                *step = weak_step;
                LOG(WARNING)
                        << "Zoom can't find a point satisfy strong wolfe condition except wolfe condition";
                return true;
            } else {
                LOG(WARNING)
                        << "Zoom pathse iteration exceed limitation, final section: ["
                        << s_lo->a << ", " << s_hi->a << "]";
                return false;
            }
        }
        if (fabs(s_hi->a - s_lo->a) < 1e-10) {
            if (weak_step > 0) {
                *step = weak_step;
                LOG(WARNING)
                        << "Zoom can't find a point satisfy strong wolfe condition except wolfe condition";
                return true;
            } else {
                LOG(WARNING)
                        << "Zoom Section: [" << s_lo->a << ", "
                        << s_hi->a << "] doesn't contain aviable step length!!";
                return false;
            }
        }
        if (current.value > cost0 + c1 * current.a * dird0
                || current.value >= s_lo->value) {
            *s_hi = current;
        } else {
            // curvature sufficient decrease condition:
            if (current.gradient >= c2 * dird0) {
                weak_step = current.a;
                if (fabs(current.gradient) <= fabs(c2 * dird0)) {
                    *step = current.a;
                    return true;
                }
            }
            if (current.gradient * (s_hi->a - s_lo->a) >= 0) {
                *s_hi = *s_lo;
            }
            *s_lo = current;
        }
        summary->line_search_iteration_count++;
    }
}

bool WolfSearch::DoSearch(const State &initial_state, Summary *summary) {
    const double &cost0 = initial_state.cost;
    const double &dird0 = initial_state.directional_derivative;
    const Vector &direction = initial_state.search_direction;
    const double &c1 = options().sufficient_decrease;
    const double &c2 = options().sufficient_curvature_decrease;
    summary->line_search_iteration_count = 1;
    Samples current;//, upper;
    Samples previous
            (0, initial_state.cost, initial_state.directional_derivative);
    const Samples unused_samples;
    function()->Evaluate(summary->initial_step, true, &current);
    double step = 0.0;
#ifdef DEBUG
    printf(">>>> initial step: %f, cost: %f, gradient: %f\n",
           current.a,
           current.value,
           current.gradient);
    plot_.SetFunc(function(), summary->initial_step);
#endif

    while (true) {
        if (current.value > cost0 + c1 * current.a * dird0 ||
                (current.value >= previous.value &&
                        summary->line_search_iteration_count > 1)) {
            if (!Zoom(initial_state, &previous, &current, &step, summary)) {
                return false;
            }
            break;
        }

        // curvature sufficient decrease condition
        if (fabs(current.gradient) <= -c2 * dird0) {
            step = current.a;
            break;
        }
        if (current.gradient >= 0) {
            if (!Zoom(initial_state, &current, &previous, &step, summary)) {
                return false;
            }
            break;
        }
        // choose step length by approximation
        double new_step = InterpolateMinimizingStepLength(
                unused_samples,
                previous,
                current,
                current.a,
                current.a * 10);
        previous = current;
        function()->Evaluate(new_step, true, &current);
        summary->line_search_iteration_count++;
    }
    summary->step = step;
    return true;
}

}
