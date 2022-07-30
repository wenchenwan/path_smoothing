//
// Created by yangt on 18-12-21.
//

#ifndef PATH_SMOOTHING_LINE_SEARCH_STEP_LENGTH_HPP
#define PATH_SMOOTHING_LINE_SEARCH_STEP_LENGTH_HPP

#include "type.hpp"
#include "minimizer.hpp"
#include "eigen_typedef.hpp"

namespace ncopt {

class StepLengthFunction {
 public:
  /**
 * value = f(X + a*d)
 * gradient  df/dX * d
 */
  struct Samples {
    Samples()
            : a(0),
              value(0),
              gradient(0),
              is_value_valid(false),
              is_gradient_valid(false) {}
    Samples(double step_length, double cost)
            : a(step_length),
              value(cost),
              gradient(0),
              is_value_valid(true),
              is_gradient_valid(false) {}
    Samples(double step_length, double cost, double grad)
            : a(step_length),
              value(cost),
              gradient(grad),
              is_value_valid(true),
              is_gradient_valid(true) {}
    double value;
    double a;
    double gradient;
    bool is_value_valid;
    bool is_gradient_valid;
    // X + a * d
    Vector vector_x;
    // df/dX(X+a*d)
    Vector vector_gradient;
  };

  StepLengthFunction(GradientProblem *problem);

  void Evaluate(double step, bool is_evaluate_gradient, Samples *output);

  inline void Init(const Vector &position, const Vector &direction) {
      position_ = position;
      direction_ = direction;
  }

 private:
  GradientProblem *problem_;
  Vector position_;
  Vector direction_;
};

class PlotFunction {
 public:
  PlotFunction();
  void SetFunc(StepLengthFunction *function, double initial_step);
  void SetApproxFunc(const Vector &poly);
  void plot();
 private:
  std::vector<double> optimal_step_;
  std::vector<double> optimal_func_;
  std::vector<double> step_;
  std::vector<double> func_;
  std::vector<double> gradient_;
  std::vector<double> approx_funx_;
};

class LineSearchStepLength {
 public:
  typedef Minimizer::State State;
  typedef StepLengthFunction::Samples Samples;

  explicit LineSearchStepLength(const LineSearchOption &option,
                                StepLengthFunction *function);

  static LineSearchStepLength *Create(const LineSearchOption &options,
                                      StepLengthFunction *function);

  double InterpolateMinimizingStepLength(const Samples &sample0,
                                         const Samples &sample1,
                                         const Samples &sample2,
                                         const double lower_step,
                                         const double upper_step);

  Vector PolynomialInterpolating(const std::vector<Samples> &samples) const;

  void FindPolynomialRoots(const Vector &polynomial, Vector *roots) const;

  virtual bool DoSearch(const State &initial_state,
                        Summary *summary) = 0;

  virtual ~LineSearchStepLength() {}

#ifdef DEBUG
  PlotFunction plot_;
#endif

 protected:
  inline const LineSearchOption &options() const {
      return options_;
  }
  inline StepLengthFunction *function() const {
      return function_;
  }

 private:
  const LineSearchOption &options_;
  StepLengthFunction *function_;
};

class ArimjoSearch : public LineSearchStepLength {
 public:
  explicit ArimjoSearch(const LineSearchOption &option,
                        StepLengthFunction *function);

  virtual bool DoSearch(const State &initial_state,
                        Summary *summary);
};

class WolfSearch : public LineSearchStepLength {
 public:
  explicit WolfSearch(const LineSearchOption &option,
                      StepLengthFunction *problem);

  virtual bool DoSearch(const State &initial_state,
                        Summary *summary);

  bool Zoom(const State &initial_state,
              Samples *s_lo,
              Samples *s_hi,
              double *step,
              Summary *summary);

};

}
#endif //PATH_SMOOTHING_LINE_SEARCH_STEP_LENGTH_HPP
