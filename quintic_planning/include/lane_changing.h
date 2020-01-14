/*
 \author  Tong Liuzhu, tongliuzhu@126.com

  \brief  lane change

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include "behavior_planner.h"
#include "behavior_state.h"

namespace autoparking_planning
{
class LaneChanging : public BehaviorState
{
   public:
    LaneChanging(int start_time, int target_lane);

    virtual TrajectorySharedPtr pathPlanning(const Eigen::VectorXd& current_state_v6,
                                             const PerceptionFrame& perception_frame,
                                             BehaviorPlanner* behavior_planner) override;

   private:
    int start_lane_;
    int target_lane_;
};
}  // end namespace
