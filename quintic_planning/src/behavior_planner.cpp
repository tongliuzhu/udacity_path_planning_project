#include <assert.h>
#include <iostream>
#include <memory>
#include "lane_keeping.h"
#include "behavior_planner.h"

namespace autoparking_planning
{
BehaviorPlanner::BehaviorPlanner() { unique_current_behavior_ = std::unique_ptr<LaneKeeping>(new LaneKeeping); }
BehaviorPlanner::~BehaviorPlanner() {}
TrajectorySharedPtr BehaviorPlanner::behaviorPlanning(const Eigen::VectorXd& curr_state_v6,
                                                      const PerceptionFrame& perception_f)
{
    TrajectorySharedPtr p_optimal_trajectory;
    p_optimal_trajectory = unique_current_behavior_->pathPlanning(curr_state_v6, perception_f, this);
    return p_optimal_trajectory;
}
}  // end namespace
