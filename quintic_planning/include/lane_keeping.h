/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  lane keeping

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once

#include "behavior_planner.h"
#include "behavior_state.h"
#include "trajectory_pool.h"

namespace autoparking_planning
{
class LaneKeeping : public BehaviorState
{
   public:
    LaneKeeping() {}
    ~LaneKeeping() override {}
    virtual TrajectorySharedPtr pathPlanning(const Eigen::VectorXd& current_state_v6,
                                             const PerceptionFrame& perception_frame,
                                             BehaviorPlanner* behavior_planner) override;
    TrajectoryPool calTrajectoryPool(const TOtherCarsTrajectory& other_trajectory,
                                     const Eigen::VectorXd& current_state_v6, const double& end_d,
                                     const double& time_duration_d);
};
}  // namespace autoparking_planning