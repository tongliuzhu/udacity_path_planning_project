#include "trajectory_pool.h"
#include <assert.h>
#include <float.h>
#include <algorithm>
#include <iostream>

namespace autoparking_planning
{
const double TrajectoryPool::kJerkCostWeight = 1.;  // 0.5;// 0.01;
const double TrajectoryPool::kAccelCostWeight = 1.;
const double TrajectoryPool::kVelocityCostWeight = 1.;
const double TrajectoryPool::kSafetyDistCostWeight = 1.;
const double TrajectoryPool::kLaneOffsetCostWeight = 1.;

TrajectoryPool::TrajectoryPool()
{
    ego_trajectory_pool_.clear();
    other_trajectories_.clear();
}
TrajectoryPool::~TrajectoryPool() {}
void TrajectoryPool::setOtherCars(const TOtherCarsTrajectory& other_trajectories)
{
    other_trajectories_ = other_trajectories;
}

void TrajectoryPool::addOtherCars(const TOtherCarsTrajectory& other_trajectories)
{
    other_trajectories_.insert(other_trajectories_.end(), other_trajectories.begin(), other_trajectories.end());
}

void TrajectoryPool::addTrajectory(TrajectorySharedPtr traj)
{
    ego_trajectory_pool_.emplace_back(traj);
    calcTrajectoryCost(traj);
}

TrajectorySharedPtr TrajectoryPool::optimalTrajectory() const
{
    TrajectorySharedPtr pTraj;
    double minCost = DBL_MAX;

    for (auto it = ego_trajectory_pool_.begin(); it != ego_trajectory_pool_.end(); ++it)
    {
        const auto cost = (*it)->getTotalCost();
        if (cost < minCost)
        {
            minCost = cost;
            pTraj = *it;
        }
    }
#if 0
    if (minCost >= Helper::kMaximumCost)
    {
        std::cout << "Something wrong! No small cost trajectory\n";
        pTraj->printInfo();
        pTraj->printDetailedInfo();
        pTraj->printCollisionPointsInfo();
        printOtherCarTrajectories();
        system("../stop.sh&");  // stop for debuging
    }
#endif
    return pTraj;
}

void TrajectoryPool::calcTrajectoryCost(TrajectorySharedPtr pTraj)
{
    double maxJs, maxJd;
    double kJerkCost = kJerkCostWeight * pTraj->calcJerkCost(Helper::kPredictionTime, maxJs, maxJd);  // hard
    const double kAccelCost = kAccelCostWeight * pTraj->calcAccelCost(Helper::kPredictionTime);       // hard
    const double kLaneOffsetCost =
        kLaneOffsetCostWeight * pTraj->calcLaneOffsetCost(Helper::kPredictionTime, pTraj->getTargetD());  // always zero

    double kVelocityCost =
        kVelocityCostWeight * pTraj->calcVelocityCost(Helper::kSpeedLimit, Helper::kPredictionTime);  // soft

    double maxSafetyDistCost = 0.;
    pTraj->clearCollisionPoints();                     // for debug
    for (const auto& otherTraj : other_trajectories_)  // otherTraj vector//[s, d, vs, vd]
    {
        const double safetyDistCost =
            pTraj->calcSafetyDistanceCost(otherTraj, Helper::kPredictionTime, pTraj->getTargetD());

        maxSafetyDistCost = std::max(maxSafetyDistCost, safetyDistCost);
    }
    double kSaferyDistCost = kSafetyDistCostWeight * maxSafetyDistCost;  // hard

    pTraj->setJerkCost(kJerkCost);
    pTraj->setAccelCost(kAccelCost);
    pTraj->setVelocityCost(kVelocityCost);
    pTraj->setSafetyDistanceCost(kSaferyDistCost);
    pTraj->setLaneOffsetCost(kLaneOffsetCost);
#if 0
pTraj->printInfo();
#endif
}

void TrajectoryPool::printOtherCarTrajectories() const
{
    const auto kPointsNum = static_cast<int>(Helper::kPredictionTime / Helper::kCostTimeStep);
    int car_num = 0;
    for (const auto& otherTraj : other_trajectories_)  // otherTraj vector//[s, d, vs, vd]
    {
        std::cout << "number car: " << car_num << "-------------------------" << std::endl;
        for (size_t i = 0; i < kPointsNum; i++)
        {
            if (i % 10 == 0) std::cout << " otherCar s, d: " << otherTraj(i, 0) << ", " << otherTraj(i, 1) << std::endl;
        }
        car_num++;
    }
}
}  // end namespace