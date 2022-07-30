//
// Created by yangt on 19-2-24.
//

#ifndef PATH_SMOOTHING_DISTANCE_FUNCTION_HPP
#define PATH_SMOOTHING_DISTANCE_FUNCTION_HPP

#include <grid_map_core/GridMap.hpp>
#include <glog/logging.h>

class DistanceFunction2D {
 public:
    DistanceFunction2D(const grid_map::GridMap &map,
                       const std::string &sdf_layer_name,
                       double threshold = 3.0)
        : sdf_(map),
          sdf_layer_(sdf_layer_name),
          th_(threshold),
          is_valid_(false) {
        if (sdf_.exists(sdf_layer_name)) {
            this->is_valid_ = true;
            LOG(INFO) << "use signed distance field.";
        }
    }

    DistanceFunction2D()
        : sdf_(grid_map::GridMap()),
          sdf_layer_(""),
          th_(0.0),
          is_valid_(false) {

    }

    inline bool isValid() const {
        return is_valid_;
    }

    inline double cost(double x, double y) const {
        const grid_map::Position pt(x, y);
        double cost = 0.0;
        if (sdf_.isInside(pt)) {
            const auto &dis = sdf_.atPosition(sdf_layer_,
                                              pt,
                                              interpolate_type);
            if (dis <= 0) {
                cost = th_ - 2*dis;
            } else if (dis <= th_) {
                cost = pow(dis - th_, 2) / th_;
            }
        }
        return cost;
    }

    inline void gradient(double x, double y, double *gradient) const {
        const grid_map::Position p_i(x, y);
        *gradient = 0;
        *(gradient + 1) = 0;
        if (sdf_.isInside(p_i)) {
            const double delta = sdf_.getResolution();
            const double half_x = sdf_.getLength().x() / 2.0;
            const double half_y = sdf_.getLength().y() / 2.0;
            const double x0 = std::max(x - delta, -half_x);
            const double y0 = std::max(y - delta, -half_y);
            const double x_ii = std::min(x + delta, half_x);
            const double y_ii = std::min(y + delta, half_y);
            *gradient = (cost(x_ii, y) - cost(x0, y)) / 2 / delta;
            *(gradient + 1) = (cost(x, y_ii) - cost(x, y0)) / 2 / delta;
        }
    }

    inline double getObstacleDistance(const grid_map::Position &pt) const {
        CHECK(sdf_.exists(dis_layer_))
        << "grid map requires a layer named 'distance' to restore the obstacle distance!";
        if (sdf_.isInside(pt)) {
            return sdf_.atPosition("distance", pt, interpolate_type);
        } else {
            return 0.0;
        }

    }

 private:
    bool is_valid_;
    const double th_;
    const std::string dis_layer_ = "distance";
    const std::string &sdf_layer_;
    const grid_map::GridMap &sdf_;
    const grid_map::InterpolationMethods
        interpolate_type = grid_map::InterpolationMethods::INTER_LINEAR;
};

#endif //PATH_SMOOTHING_DISTANCE_FUNCTION_HPP
