/*
 \author  Tong Liuzhu, tongliuzhu@126.com

  \brief  EgoCar class for the ego vehicle data and processing

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include <memory>
#include "behavior_planner.h"
#include "common_define.h"
#include "perception_frame.h"

namespace autoparking_planning
{
class FrenetMap;

class EgoCar
{
   public:
    EgoCar(const FrenetMap& frenet_map);
    ~EgoCar();
    void updateTrajectory(CarLocalizationData& current_state, std::vector<SensorFusionData>& sensor_fusion_data,
                          const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);

    inline std::vector<double>& get_next_x_vals() { return next_x_pos_; }
    inline std::vector<double>& get_next_y_vals() { return next_y_pos_; }
   private:
    inline void setCurrentState(const Eigen::VectorXd& current_state) { current_frenet_state_ = current_state; }
    void setPerceptionFrameData(std::vector<SensorFusionData>& sensor_fusion_data);

   private:
    BehaviorPlanner ego_behavior_planner_;
    PerceptionFrame perception_frame_;
    const FrenetMap& frenet_map_;
    std::vector<double> next_x_pos_;        //[next_x, next_y]
    std::vector<double> next_y_pos_;        //[next_x, next_y]
    Eigen::VectorXd current_frenet_state_;  //[s, s_s, s_ss, d, d_d, d_dd] in frenet

    // specially for Udacity simulator
    const double prediction_trajectory_time_ = 2.;
    // in general reactive should not be too small (should be larger than the latency time)
    // it also should not be too large since these point won't be change for two consecutive iteration
    const double reactive__trajectory_time_ = 0.3;  // points_num = /0.02;
};
}  // end namespace