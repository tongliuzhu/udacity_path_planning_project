/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  behavior states

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once

#include "other_car.h"
#include "perception_frame.h"
#include "trajectory_planner.h"
//#include "behavior_planner.h"

namespace autoparking_planning
{
class BehaviorPlanner;
class BehaviorState
{
   public:
    BehaviorState() {}
    virtual ~BehaviorState() {}
    virtual TrajectorySharedPtr pathPlanning(const Eigen::VectorXd& current_state_v6,
                                             const PerceptionFrame& perception_frame,
                                             BehaviorPlanner* behavior_planner) = 0;
};
}  // end namespace
