//
// Created by yangt on 19-2-18.
//

#ifndef PATH_SMOOTHING_PATH_SMOOTHING_HPP
#define PATH_SMOOTHING_PATH_SMOOTHING_HPP
#ifdef GPMP2_SMOOTHING_ENABLE
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gpmp2/dynamics/VehicleDynamicsFactorPose2.h>
#include <gpmp2/gp/GaussianProcessPriorPose2.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
// #include "path_smoothing/obstacle_factor.hpp"
#endif
#include <geometry_msgs/Point.h>
#include "tiny_spline/tinysplinecpp.h"
#include "path_smoothing/cg_smoothing_function.hpp"
#include "non_constrained_optimiztion/gradient_problem_solve.hpp"
//#include "path_smoothing_unit.hpp"

namespace path_smoothing {

class PathSmoothing {
 public:
    struct Options {
        SmootherType smoother_type = CONJUGATE_GRADIENT_METHOD;
        // options for conjugate gradient method
        double cg_heading_term_coe = 1;
        double cg_curvature_term_coe = 1.0;
        double cg_obstacle_term_coe = 2.0;
        DifferenceType cg_difference_type = CPPAD;
        NonlinearSolverType cg_solver = CERES_SOLVER;
        // options for gp method
        double gp_obs_sigma = 0.05;
        double gp_vehicle_dynamic_sigma = 0.00;
        double gp_dt = 0.5;
        LeastSquaresSolver gp_solver = LEVENBERG_MARQUARDT;
        // options for non-derivative method
        double lower_boundary = 1.0;
        double safe_margin = 0.2;
        double max_curvature = 0.4;

        // options for signed distance field
        DistanceFunction2D *function = nullptr;
    };

    PathSmoothing(const int path_size);

//    template<class PathElement>
//    static PathSmoothing *createSmoother(const Options &options,
//                                         const std::vector<PathElement> &path);
    static PathSmoothing *createSmoother(const Options &options,
                                         const std::vector<geometry_msgs::Point> &path);

//    template<class PathElement>
//    void getSmoothPath(std::vector<PathElement> *path) const;
    void getSmoothPath(std::vector<geometry_msgs::Point> *path,std::vector<geometry_msgs::Point> *control) const;

    inline const int &pathSize() const {
        return path_size_;
    }

    virtual void smoothPath(const Options &options) = 0;
    virtual double x(int i) const = 0;
    virtual double y(int i) const = 0;

    virtual ~PathSmoothing() {}

 private:
    const int path_size_;
};

class CgSmoothing : public PathSmoothing {
 public:

//    template<class PathElement>
//    CgSmoothing(const Options &options, const std::vector<PathElement> &path);
    CgSmoothing(const Options &options,
                const std::vector<geometry_msgs::Point> &path);

    virtual double x(int i) const;

    virtual double y(int i) const;

    virtual void smoothPath(const Options &options);
 private:
    CgSmoothingFunction::Settings settings_;
    CgSmoothingFunction::Vector params_;
};

#ifdef GPMP2_SMOOTHING_ENABLE
class GpSmoothing : public PathSmoothing {
 public:
//  template<class PathElement>
//  GpSmoothing(const Options &options, const std::vector<PathElement> &path);

    GpSmoothing(const Options &options,
                const std::vector<geometry_msgs::Point> &path);

    virtual void smoothPath(const Options &options);

    virtual double x(int i) const;
    virtual double y(int i) const;

 private:
    gtsam::Values result_;
    gtsam::Values initial_guess;
    gtsam::NonlinearFactorGraph graph_;
};
#endif

}

//#include "path_smoothing-in.hpp"

#endif //PATH_SMOOTHING_PATH_SMOOTHING_HPP
