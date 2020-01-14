/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  trajectory planner

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include "common_define.h"
#include "helper.h"

namespace autoparking_planning
{
class TrajectoryPlanner;
typedef std::shared_ptr<TrajectoryPlanner> TrajectorySharedPtr;

class TrajectoryPlanner
{
   public:
    TrajectoryPlanner();

    // Input:
    //   start_state_v6 is a vector of size 6: [s, s_d, s_dd, d, d_d, d_dd]
    //   end_state_v6 is a vector of size 6:   [s, s_d, s_dd, d, d_d, d_dd]
    TrajectoryPlanner(const Eigen::VectorXd& start_state_v6, const Eigen::VectorXd& end_state_v6, double duration_s,
                      double duration_d);

    // calculates Jerk Minimizing Trajectory for the Velocity Lane keeping state, using currect state,
    // target velocity and
    // duration T.
    // curr_state_v6 is 6-dim vector: [s, s_d, s_dd, d, d_d, d_dd].
    // Usually only the first 3 coord. [s, s_d, s_dd] is used for trajectory calculation (it is better
    // to split to
    // S-trajectory and D-trajectory).
    static std::shared_ptr<TrajectoryPlanner> velocityKeepingSTTrajectory(const Eigen::VectorXd& curr_state_v6,
                                                                          double end_d, double target_velocity,
                                                                          double time_duration_s,
                                                                          double time_duration_d);

    // Returns vector state [s, s_d, s_dd, d, d_d, d_dd] at the time 'time'.
    Eigen::VectorXd evaluateStateAt(const double& time) const;

    // calculation of cost functions.
    // calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d,
    // s_dd, d, d_d, d_dd].
    double calcJerkCost(const double& time_duration, double& maxJs, double& maxJd) const;
    double calcAccelCost(const double& time_duration) const;
    double calcVelocityCost(const double& target_velocity, const double& time_duration) const;
    double calcSafetyDistanceCost(const Eigen::MatrixXd& s2, const double& time_duration, const double& target_d);
    double calcLaneOffsetCost(const double& time_duration, const double& target_d);
    void printInfo();
    void printDetailedInfo();
    std::pair<double, double> minMaxVelocityS() const;

    // Returns trajectory duration in seconds.
    inline double getDurationS() const { return duration_s_; }
    inline double getDurationD() const { return duration_d_; }
    inline Eigen::VectorXd getStartState() const { return start_state_; }
    inline double getTargetS() const { return end_state_(0); }
    inline double getTargetD() const { return end_state_(3); }
    inline double getStartD() const { return start_state_(3); }
    inline double getTotalCost() const { return cost_.sum(); }
    inline void setJerkCost(const double& cost) { cost_(0) = cost; }
    inline void setAccelCost(const double& cost) { cost_(1) = cost; }
    inline void setVelocityCost(const double& cost) { cost_(2) = cost; }
    inline void setSafetyDistanceCost(const double& cost) { cost_(3) = cost; }
    inline void setLaneOffsetCost(const double& cost) { cost_(4) = cost; }
    inline double getJerkCost() const { return cost_(0); }
    inline double getAccelCost() const { return cost_(1); }
    inline double getVelocityCost() const { return cost_(2); }
    inline double getSafetyDistanceCost() const { return cost_(3); }
    inline double getLaneOffsetCost() const { return cost_(4); }
    inline double getTargetVelocity() const { return target_velocity_; }
    inline Eigen::VectorXd getSCoeffs() const { return s_coeffs_; }
    inline Eigen::VectorXd getDCoeffs() const { return d_coeffs_; }
    double calcOnePointLaneOffsetCost(const double& d);
    void clearCollisionPoints();
    void printCollisionPointsInfo();

   private:
    double calcOnePointSafetyDistanceCost(const double& Sdist, const double& Ddist, const double& velocity);

    // calculates Jerk Minimizing Trajectory for start state, end state and duration T.
    // Input:
    // start_state_v3 is vector of size 3: [s, s_d, s_dd]
    // end_state_v3 is vector of size 3:   [s, s_d, s_dd]
    // Returns:
    // vector of size 6: [s, s_d, s_dd, a3, a4, a5]
    static Eigen::VectorXd calcQuinticPolynomialCoeffs(const Eigen::VectorXd& start_state_v3,
                                                       const Eigen::VectorXd& end_state_v3, const double& T);

    // calculates polynomial value at time 't' and returns 3-dim vector: [s, s_d, s_dd]
    static Eigen::VectorXd calcPolynomialAt(const Eigen::VectorXd& coeffsX6, const double& t);
    // Returns Jerk value using polynomial value at time 't'.
    inline double calPolynominalJerkAt(const Eigen::VectorXd& coeffsX6, const double& t) const;
    inline double calPolynominalAccAt(const Eigen::VectorXd& coeffsX6, const double& t) const;
    inline double calPolynomialVelAt(const Eigen::VectorXd& coeffsX6, const double& t) const;
    double calPolynominalDistanceAt(const Eigen::VectorXd& coeffsX6, const double& t) const;

    // calculate Jerk optimal polynomial for S-trajectory with keeping velocity, using current S
    // start-state [s, s_d,
    // s_dd],
    // target velocity m/s and specified duration T in sec.
    // Returns: polynomial copeffs as 6-dim vector.
    static Eigen::VectorXd calcSPolynomialVelocityKeeping(const Eigen::VectorXd& start_state_V3,
                                                          const double& target_velocity, const double& T);

   private:
    double duration_s_;  // duration T of S state in secs
    double duration_d_;  // duration T of D state in secs
    double target_velocity_;

    Eigen::VectorXd cost_;
    Eigen::VectorXd start_state_;                      // [s, s_dot, s_ddot, d, d_dot, d_ddot]
    Eigen::VectorXd end_state_;                        // [s, s_dot, s_ddot, d, d_dot, d_ddot]
    Eigen::VectorXd s_coeffs_;                         // 6 coeffs of quintic polynomial
    Eigen::VectorXd d_coeffs_;                         // 6 coeffs of quintic polynomial
    std::vector<Point2D> ego_car_collision_points_;    // only for debug
    std::vector<Point2D> other_car_collision_points_;  // only for debug
};

/////////////////////////////////////////////////////////////////////////////////////////

inline double TrajectoryPlanner::calPolynominalJerkAt(const Eigen::VectorXd& a, const double& t) const
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    // calculate s_ddd(t)
    return 6 * a(3) + 24 * a(4) * t + 60 * a(5) * t * t;
}

inline double TrajectoryPlanner::calPolynominalAccAt(const Eigen::VectorXd& a, const double& t) const
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    const auto t2 = t * t;
    const auto t3 = t2 * t;

    // calculate s_dd(t)
    return 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t2 + 20 * a(5) * t3;
}

inline double TrajectoryPlanner::calPolynomialVelAt(const Eigen::VectorXd& a, const double& t) const
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;

    // calculate s_d(t)
    return a(1) + 2. * a(2) * t + 3. * a(3) * t2 + 4. * a(4) * t3 + 5. * a(5) * t4;
}

inline double TrajectoryPlanner::calPolynominalDistanceAt(const Eigen::VectorXd& a, const double& t) const
{
    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    const auto t5 = t4 * t;

    // calculate s_d(t)
    return a(0) + a(1) * t + a(2) * t2 + a(3) * t3 + a(4) * t4 + a(5) * t5;
}
}  // end namespace