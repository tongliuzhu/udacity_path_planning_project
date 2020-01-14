/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  behavior of the egocar

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include "behavior_state.h"
#include "trajectory_planner.h"

namespace autoparking_planning
{
class PerceptionFrame;
class BehaviorPlanner
{
   public:
    BehaviorPlanner();
    ~BehaviorPlanner();

    // current_state_v6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
    TrajectorySharedPtr behaviorPlanning(const Eigen::VectorXd& current_state_v6, const PerceptionFrame& p_frame);
    // common for each states
    inline void setBehavior(BehaviorState* new_behavior) { unique_current_behavior_.reset(new_behavior); };
   private:
    std::unique_ptr<BehaviorState> unique_current_behavior_;
};
}  // end namespace
