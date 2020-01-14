/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  trajectory pool

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once

#include <iostream>
#include <limits>
#include <memory>

#include "helper.h"
#include "other_car.h"
#include "trajectory_planner.h"

namespace autoparking_planning
{
typedef std::vector<OtherCar> TOtherVehicles;
typedef std::vector<Eigen::MatrixXd> TOtherCarsTrajectory;
typedef std::vector<TrajectorySharedPtr> VectorTrajPool;

class TrajectoryPool
{
   public:
    TrajectoryPool();
    ~TrajectoryPool();
    // set other cars trajecotry
    void setOtherCars(const TOtherCarsTrajectory& otherTrajectories);
    // add other cars trajecotry
    void addOtherCars(const TOtherCarsTrajectory& otherTrajectories);
    void addTrajectory(TrajectorySharedPtr traj);
    // get optimal trajectory of all the trajectories
    TrajectorySharedPtr optimalTrajectory() const;
    void printOtherCarTrajectories() const;
    inline VectorTrajPool& getEgoTrajectoryPool() { return ego_trajectory_pool_; }
    inline TOtherCarsTrajectory& getOtherTrajectoryPool() { return other_trajectories_; }
   private:
    void calcTrajectoryCost(TrajectorySharedPtr traj);

   private:
    VectorTrajPool ego_trajectory_pool_;
    TOtherCarsTrajectory other_trajectories_;

    static const double kJerkCostWeight;
    static const double kAccelCostWeight;
    static const double kVelocityCostWeight;
    static const double kSafetyDistCostWeight;
    static const double kLaneOffsetCostWeight;
};
}  // end namespace
