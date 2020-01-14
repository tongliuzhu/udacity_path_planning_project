#include <vector>
#include "lane_keeping.h"
#include "trajectory_pool.h"
#include "lane_changing.h"

namespace autoparking_planning
{
LaneChanging::LaneChanging(int start_Lane, int target_lane) : start_lane_(start_Lane), target_lane_(target_lane) {}
TrajectorySharedPtr LaneChanging::pathPlanning(const Eigen::VectorXd& current_state_v6,
                                               const PerceptionFrame& perception_frame,
                                               BehaviorPlanner* behavior_planner)
{
    const double kCurrentD = current_state_v6(3);
    const double kTargetD = Helper::laneNumberToD(target_lane_);

    TrajectoryPool trajectory_pool;
    trajectory_pool.setOtherCars(
        perception_frame.getLeadingCarsTrajectoryInLane(current_state_v6, start_lane_, Helper::kPredictionTime));
    trajectory_pool.addOtherCars(
        perception_frame.getOtherCarsTrajectoryInLane(current_state_v6, target_lane_, Helper::kPredictionTime));
    for (double target_v = 0; target_v < Helper::kSpeedLimit; target_v += 1)
    {
        for (double target_t = 1; target_t <= Helper::kPredictionTime; target_t += 1)
        {
            trajectory_pool.addTrajectory(TrajectoryPlanner::velocityKeepingSTTrajectory(
                current_state_v6, kTargetD, target_v, target_t, Helper::kChangeLaneTime));
        }
    }
    TrajectorySharedPtr pOptimalTraj = trajectory_pool.optimalTrajectory();
    const double kReachTargetDTolerance = 0.1;
    if (std::abs(kTargetD - kCurrentD) <= kReachTargetDTolerance)
    {
        std::cout << " Remove Change Lane State " << std::endl;
        behavior_planner->setBehavior(new LaneKeeping);
    }
    return pOptimalTraj;
}
}  // end namespace
