#include "lane_keeping.h"
#include <future>
#include <iostream>
#include "lane_changing.h"
#include "trajectory_pool.h"
#define NOT_USING_MULTITHREADS false

namespace autoparking_planning
{
TrajectorySharedPtr LaneKeeping::pathPlanning(const Eigen::VectorXd& current_state_v6,
                                              const PerceptionFrame& perception_frame,
                                              BehaviorPlanner* behavior_planner)
{
    const bool enable_lane_change = true;
    const bool enable_acc = true;
    const double kCurrentS = current_state_v6(0);
    const double kCurrentD = current_state_v6(3);
    const int kCurrentLane = Helper::dToLaneNumber(kCurrentD);
    TrajectoryPool trajectory_pool;
#if NOT_USING_MULTITHREADS
    // Start state: [s0, s0_d, s0_dd] at t0
    // End state:   [s1_d, s1_dd] at t1 = t0 + T
    // st_d = target velocity.
    // Task: generate optimal longitudinal trajectory set of quartic polynomials by
    // varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d +
    // ∆s_d[i]], 0, T[j] ]
    TOtherCarsTrajectory cars_on_the_lane_traj =
        perception_frame.getLeadingCarsTrajectoryInLane(current_state_v6, kCurrentLane, Helper::kPredictionTime);
    trajectory_pool.setOtherCars(cars_on_the_lane_traj);
    // Generates trajectory to stay in the lane.
    if (enable_acc)
    {
        for (double target_v = 0; target_v <= Helper::kSpeedLimit; target_v += 1)
        {
            for (double T = 1; T <= Helper::kPredictionTime; T += 1)
            {
                trajectory_pool.addTrajectory(
                    TrajectoryPlanner::velocityKeepingSTTrajectory(current_state_v6, kCurrentD, target_v, T, 0));
            }
        }
    }

    if (enable_lane_change)
    {
        // Changing lane to right trajectory
        if (kCurrentLane + 1 <= Helper::kMaximumLaneNum)
        {
            const int kTargetLane = kCurrentLane + 1;
            const double kTargetD = Helper::laneNumberToD(kTargetLane);
            trajectory_pool.addOtherCars(
                perception_frame.getOtherCarsTrajectoryInLane(current_state_v6, kTargetLane, Helper::kPredictionTime));

            for (double target_v = 0; target_v <= Helper::kSpeedLimit; target_v += 1)
            {
                for (double T = 1; T <= Helper::kPredictionTime; T += 1)
                {
                    TrajectorySharedPtr p_traj = TrajectoryPlanner::velocityKeepingSTTrajectory(
                        current_state_v6, kTargetD, target_v, T, Helper::kChangeLaneTime);
                    trajectory_pool.addTrajectory(p_traj);
                }
            }
        }
        // Changing lane to left trajectory
        if (kCurrentLane - 1 >= Helper::kMinimumLaneNum)
        {
            const int kTargetLane = kCurrentLane - 1;
            const double kTargetD = Helper::laneNumberToD(kTargetLane);
            trajectory_pool.addOtherCars(
                perception_frame.getOtherCarsTrajectoryInLane(current_state_v6, kTargetLane, Helper::kPredictionTime));

            for (double target_v = 0; target_v <= Helper::kSpeedLimit; target_v += 1)
            {
                for (double T = 1; T <= Helper::kPredictionTime; T += 1)
                {
                    TrajectorySharedPtr pTraj = TrajectoryPlanner::velocityKeepingSTTrajectory(
                        current_state_v6, kTargetD, target_v, T, Helper::kChangeLaneTime);
                    trajectory_pool.addTrajectory(pTraj);
                }
            }
        }
    }
#else
    TOtherCarsTrajectory cars_on_the_lane_traj =
        perception_frame.getLeadingCarsTrajectoryInLane(current_state_v6, kCurrentLane, Helper::kPredictionTime);
    trajectory_pool.setOtherCars(cars_on_the_lane_traj);
    std::future<std::unique_ptr<TrajectoryPool> > fu_lane_keeping, fu_lane_plus, fu_lane_minus;
    // Generates trajectory to stay in the lane.
    if (enable_acc)
    {
        const double kDurationD = 0.0;
        const double kChangeLaneTimeInLaneKeeping = 0.;
        TOtherCarsTrajectory other_trajectory_lane_keeping = trajectory_pool.getOtherTrajectoryPool();
        fu_lane_keeping = std::async(std::launch::async, &LaneKeeping::calTrajectoryPool, this,
                                     std::move(other_trajectory_lane_keeping), std::cref(current_state_v6),
                                     std::cref(kCurrentD), std::cref(kChangeLaneTimeInLaneKeeping));
    }

    if (enable_lane_change)
    {
        // Changing lane to right trajectory
        if (kCurrentLane + 1 <= Helper::kMaximumLaneNum)
        {
            const int kTargetLane = kCurrentLane + 1;
            const double kTargetD = Helper::laneNumberToD(kTargetLane);
            trajectory_pool.addOtherCars(
                perception_frame.getOtherCarsTrajectoryInLane(current_state_v6, kTargetLane, Helper::kPredictionTime));
            TOtherCarsTrajectory other_trajectory_lane_plus = trajectory_pool.getOtherTrajectoryPool();
            fu_lane_plus = std::async(std::launch::async, &LaneKeeping::calTrajectoryPool, this,
                                      std::move(other_trajectory_lane_plus), std::cref(current_state_v6),
                                      std::move(kTargetD), std::cref(Helper::kChangeLaneTime));
        }
        // Changing lane to left trajectory
        if (kCurrentLane - 1 >= Helper::kMinimumLaneNum)
        {
            const int kTargetLane = kCurrentLane - 1;
            const double kTargetD = Helper::laneNumberToD(kTargetLane);
            trajectory_pool.addOtherCars(
                perception_frame.getOtherCarsTrajectoryInLane(current_state_v6, kTargetLane, Helper::kPredictionTime));
            TOtherCarsTrajectory other_trajectory_lane_minus = trajectory_pool.getOtherTrajectoryPool();
            fu_lane_minus = std::async(std::launch::async, &LaneKeeping::calTrajectoryPool, this,
                                       std::move(other_trajectory_lane_minus), std::cref(current_state_v6),
                                       std::move(kTargetD), std::cref(Helper::kChangeLaneTime));
        }
    }
    // add all the trajectories to trajectory_pool
    if (enable_acc)
    {
        std::unique_ptr<TrajectoryPool> ego_lane_keeping_trajectory_pool =
            std::move(fu_lane_keeping.get());  // must get first and then use
        trajectory_pool.getEgoTrajectoryPool().insert(trajectory_pool.getEgoTrajectoryPool().end(),
                                                      ego_lane_keeping_trajectory_pool->getEgoTrajectoryPool().begin(),
                                                      ego_lane_keeping_trajectory_pool->getEgoTrajectoryPool().end());
    }
    if (enable_lane_change && kCurrentLane + 1 <= Helper::kMaximumLaneNum)
    {
        std::unique_ptr<TrajectoryPool> ego_lane_plus_trajectory_pool = std::move(fu_lane_plus.get());
        trajectory_pool.getEgoTrajectoryPool().insert(trajectory_pool.getEgoTrajectoryPool().end(),
                                                      ego_lane_plus_trajectory_pool->getEgoTrajectoryPool().begin(),
                                                      ego_lane_plus_trajectory_pool->getEgoTrajectoryPool().end());
    }
    if (enable_lane_change && kCurrentLane - 1 >= Helper::kMinimumLaneNum)
    {
        std::unique_ptr<TrajectoryPool> ego_lane_minus_trajectory_pool = std::move(fu_lane_minus.get());
        trajectory_pool.getEgoTrajectoryPool().insert(trajectory_pool.getEgoTrajectoryPool().end(),
                                                      ego_lane_minus_trajectory_pool->getEgoTrajectoryPool().begin(),
                                                      ego_lane_minus_trajectory_pool->getEgoTrajectoryPool().end());
    }
#endif
    TrajectorySharedPtr p_optimal_traj = trajectory_pool.optimalTrajectory();
    assert(p_optimal_traj != nullptr);  // optimal trajectory should not be empty
    const auto kTargetDFinal = p_optimal_traj->getTargetD();
    static TrajectorySharedPtr pre_optimal_traj = p_optimal_traj;

    if (kCurrentD != kTargetDFinal)
    {
        std::cout << " Add Change Lane State " << std::endl;
        behavior_planner->setBehavior(new LaneChanging(kCurrentLane, Helper::dToLaneNumber(kTargetDFinal)));
#if 0
	std::cout << "------- Best ----------" << std::endl;
	std::cout << " C: " << p_optimal_traj->getTotalCost()
		<< " DC: " << p_optimal_traj->getSafetyDistanceCost()
		<< " VC: " << p_optimal_traj->getVelocityCost()
		<< " JC: " << p_optimal_traj->getJerkCost()
		<< std::endl;
	std::cout << " C_pre: " << pre_optimal_traj->getTotalCost()
		<< " DC_pre: " << pre_optimal_traj->getSafetyDistanceCost()
		<< " VC_pre: " << pre_optimal_traj->getVelocityCost()
		<< " JC_pre: " << pre_optimal_traj->getJerkCost()
		<< std::endl;
#endif
    }
    pre_optimal_traj = p_optimal_traj;
    return p_optimal_traj;
}

std::unique_ptr<TrajectoryPool> LaneKeeping::calTrajectoryPool(const TOtherCarsTrajectory& other_trajectory,
                                                               const Eigen::VectorXd& current_state_v6,
                                                               const double& end_d, const double& time_duration_d)
{
    std::unique_ptr<TrajectoryPool> temp_trajectory_pool = make_unique<TrajectoryPool>();
    TrajectoryPlanner trajectory_p;
    temp_trajectory_pool->setOtherCars(other_trajectory);
    for (double target_v = 0; target_v <= Helper::kSpeedLimit; target_v += 1)
    {
        for (double T = 1; T <= Helper::kPredictionTime; T += 1)
        {
            temp_trajectory_pool->addTrajectory(
                trajectory_p.velocityKeepingSTTrajectory(current_state_v6, end_d, target_v, T, time_duration_d));
        }
    }
    return temp_trajectory_pool;
}
}  // namespace autoparking_planning