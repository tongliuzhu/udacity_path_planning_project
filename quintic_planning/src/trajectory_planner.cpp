#include "trajectory_planner.h"
#include <assert.h>
#include <assert.h>
#include <limits>

// using namespace std;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace autoparking_planning
{
///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectoryPlanner::TrajectoryPlanner()
    : duration_s_(0.),
      duration_d_(0.),
      target_velocity_(0.),
      cost_(VectorXd::Zero(5)),
      start_state_(VectorXd::Zero(6)),
      end_state_(VectorXd::Zero(6)),
      s_coeffs_(VectorXd::Zero(6)),
      d_coeffs_(VectorXd::Zero(6))
{
    ego_car_collision_points_.clear();
    other_car_collision_points_.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectoryPlanner::TrajectoryPlanner(const VectorXd &start_state_v6, const VectorXd &end_state_v6, double duration_s,
                                     double duration_d)
    : duration_s_(duration_s),
      duration_d_(duration_d),
      target_velocity_(0.),
      start_state_(start_state_v6),
      end_state_(end_state_v6),
      s_coeffs_(VectorXd::Zero(6)),
      d_coeffs_(VectorXd::Zero(6))
{
    s_coeffs_ = calcQuinticPolynomialCoeffs(start_state_v6.head(3), end_state_v6.head(3), duration_s_);
    d_coeffs_ = calcQuinticPolynomialCoeffs(start_state_v6.segment(3, 3), end_state_v6.segment(3, 3), duration_d_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
TrajectorySharedPtr TrajectoryPlanner::velocityKeepingSTTrajectory(const VectorXd &curr_state_v6, double end_d,
                                                                   double target_velocity, double time_duration_s,
                                                                   double time_duration_d)
{
    TrajectorySharedPtr pTraj = std::make_shared<TrajectoryPlanner>();
    pTraj->duration_s_ = time_duration_s;
    pTraj->duration_d_ = time_duration_d;
    pTraj->start_state_ = curr_state_v6;
    pTraj->end_state_(3) = end_d;
    pTraj->target_velocity_ = target_velocity;
    const double currD = curr_state_v6(3);
    pTraj->s_coeffs_ = calcSPolynomialVelocityKeeping(curr_state_v6.head(3), target_velocity, time_duration_s);

    if (currD == end_d)
    {
        pTraj->d_coeffs_(0) = curr_state_v6(3);  // saves D position on the road.
    }
    else
    {
        pTraj->d_coeffs_ = calcQuinticPolynomialCoeffs(pTraj->start_state_.segment(3, 3),
                                                       pTraj->end_state_.segment(3, 3), time_duration_d);
    }
    return pTraj;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd TrajectoryPlanner::evaluateStateAt(const double &time) const
{
    double t = time;
    VectorXd stateS = calcPolynomialAt(s_coeffs_, std::min(t, duration_s_));
    VectorXd stateD = calcPolynomialAt(d_coeffs_, std::min(t, duration_d_));
    // model with acceleration = 0
    if (t > duration_s_)
    {
        stateS(0) += stateS(1) * (t - duration_s_);
    }

    if (t > duration_d_)
    {
        stateD(0) += stateD(1) * (t - duration_d_);
    }
    VectorXd state(6);
    state << stateS, stateD;
    return state;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd TrajectoryPlanner::calcQuinticPolynomialCoeffs(const VectorXd &startStateX3, const VectorXd &endStateX3,
                                                        const double &T)
{
    // Calculate the Jerk Minimizing Trajectory that connects the initial state
    // to the final state in time T.

    // INPUTS
    // start - the vehicles start location given as a length three array
    //     corresponding to initial values of [s, s_dot, s_double_dot]

    // end   - the desired end state for vehicle. Like "start" this is a
    //     length three array.

    // T     - The duration, in seconds, over which this maneuver should occur.

    // OUTPUT
    // an array of length 6, each value corresponding to a coefficent in the polynomial
    // s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    const auto s_i = startStateX3[0];
    const auto s_i_d = startStateX3[1];
    const auto s_i_dd = startStateX3[2];

    const auto s_f = endStateX3[0];
    const auto s_f_d = endStateX3[1];
    const auto s_f_dd = endStateX3[2];

    const auto T2 = T * T;
    const auto T3 = T2 * T;
    const auto T4 = T3 * T;
    const auto T5 = T4 * T;

    MatrixXd A = MatrixXd(3, 3);
    A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

    VectorXd b = VectorXd(3);
    b << s_f - (s_i + s_i_d * T + 0.5 * s_i_dd * T2), s_f_d - (s_i_d + s_i_dd * T), s_f_dd - s_i_dd;

    Vector3d x = A.colPivHouseholderQr().solve(b);

    VectorXd coeffs = VectorXd(6);
    coeffs << s_i, s_i_d, 0.5 * s_i_dd, x[0], x[1], x[2];
    return coeffs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd TrajectoryPlanner::calcPolynomialAt(const VectorXd &coeffsX6, const double &t)
{
    const VectorXd &a = coeffsX6;

    // s(t) = a0 + a1 * t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = a1 + 2*a2 * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = 2*a2 + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    // 3d Vector: [s, s_d, s_dd]
    VectorXd state = VectorXd::Zero(3);

    const auto t2 = t * t;
    const auto t3 = t2 * t;
    const auto t4 = t3 * t;
    const auto t5 = t4 * t;

    // calculate s(t)
    state(0) = a(0) + a(1) * t + a(2) * t2 + a(3) * t3 + a(4) * t4 + a(5) * t5;

    // calculate s_d(t)
    state(1) = a(1) + 2 * a(2) * t + 3 * a(3) * t2 + 4 * a(4) * t3 + 5 * a(5) * t4;

    // calculate s_dd(t)
    state(2) = 2 * a(2) + 6 * a(3) * t + 12 * a(4) * t2 + 20 * a(5) * t3;

    return state;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd TrajectoryPlanner::calcSPolynomialVelocityKeeping(const Eigen::VectorXd &startStateV3,
                                                                  const double &target_velocity, const double &T)
{
    // Calculate 6 coeffs [a1, a2, a3, a4, a5] with constraints: a4=0 and a5 = 0.

    // Using constraints:  start [S_0, S_d, S_dd]
    //                       end [S_f_target_d, S_f_dd=0, T_f]
    //   and
    //
    // s(t) = s0 + s0_d * t + 0.5 * s0_dd * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5
    // s_d(t) = s0_d + s0_dd * t + 3*a3 * t^2 + 4*a4 * t^3 + 5*a5 * t^4
    // s_dd(t) = s0_dd + 6*a3 * t^1 + 12*a4 * t^2 + 20*a5 * t^3

    // S_0_d + S_0_dd*T + 3*a3*T^2 + 4*a4*T^3 = S_target_d
    // S_0_dd + 6*a3*T + 12*a4*T^2 = 0

    //  | 3T^2  4T^3 |   | a3 |   | S_f_target_d - (S_0_d + S_0_dd * T) |
    //  |            | x |    | = |                                     |
    //  | 6T   12T^2 |   | a4 |   | -S_0_dd                             |

    const double T2 = T * T;
    const double T3 = T2 * T;

    MatrixXd A(2, 2);
    A << 3 * T2, 4 * T3, 6 * T, 12 * T2;

    VectorXd b(2);
    b << target_velocity - startStateV3(1) - startStateV3(2) * T, 0. - startStateV3(2);

    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    assert(x(0) == x(0));
    assert(x(1) == x(1));

    VectorXd S_Coeffs(6);
    S_Coeffs << startStateV3(0), startStateV3(1), 0.5 * startStateV3(2), x, 0.;
    return S_Coeffs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Calculate Jerk cost function for evaluated trajectory points as array of vectors like [s, s_d,
// s_dd, d, d_d, d_dd].
double TrajectoryPlanner::calcJerkCost(const double &timeDuration, double &maxJs, double &maxJd) const
{
    double total_jert_cost = 0.;
    for (double t = 0; t < timeDuration; t += Helper::kCostTimeStep)
    {
        const double kJerkS = calPolynominalJerkAt(s_coeffs_, std::min(t, duration_s_));
        const double kJerkD = calPolynominalJerkAt(d_coeffs_, std::min(t, duration_d_));
        if (sqrt(kJerkS * kJerkS + kJerkD * kJerkD) >= Helper::kMaximumJerk)
        {
            total_jert_cost += Helper::kMaximumCost;  // within Maximum Jerk Limit
        }
    }
    return total_jert_cost;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double TrajectoryPlanner::calcAccelCost(const double &timeDuration) const
{
    double total_acc_cost = 0.;
    for (double t = 0; t < timeDuration; t += Helper::kCostTimeStep)
    {
        const double kAccS = calPolynominalAccAt(s_coeffs_, std::min(t, duration_s_));
        const double kAccD = calPolynominalAccAt(d_coeffs_, std::min(t, duration_d_));
        if (sqrt(kAccS * kAccS + kAccD * kAccD) >= Helper::kMaximumAcc)
        {
            total_acc_cost += Helper::kMaximumCost;  // within Maximum Acc Limit
        }
    }
    return total_acc_cost;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<double, double> TrajectoryPlanner::minMaxVelocityS() const
{
    const double kMaxDoubleVal = 1e10;
    const double kMinDoubleVal = 1e-10;
    std::pair<double, double> minMax(kMaxDoubleVal, kMinDoubleVal);

    for (double t = 0; t < duration_s_; t += Helper::kCostTimeStep)
    {
        const double v = calPolynomialVelAt(s_coeffs_, t);
        minMax.first = std::min(v, minMax.first);
        minMax.second = std::max(v, minMax.second);
    }

    return minMax;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double TrajectoryPlanner::calcVelocityCost(const double &target_velocity, const double &timeDuration) const
{
    // try to maintain the target vel, do not acc or decellerate too much.
    double cost = 0;
    double t = 0;
    const int kPointsNum = static_cast<int>(timeDuration / Helper::kCostTimeStep);
    bool too_large_speed_flag = false;

    for (int i = 0; i < kPointsNum; i++)
    {
        const double kVelocityS = calPolynomialVelAt(s_coeffs_, std::min(t, duration_s_));
        const double kVelocityD = calPolynomialVelAt(d_coeffs_, std::min(t, duration_d_));
        double v = std::sqrt(kVelocityS * kVelocityS + kVelocityD * kVelocityD);

        if (v >= Helper::kSpeedLimit || kVelocityS < 0)
        {
            too_large_speed_flag = true;
            cost += Helper::kMaximumCost;
        }
        else
        {
            cost += Helper::distance(Helper::kSpeedLimit, kVelocityS);  // possible larger velocity better
        }
        t += Helper::kCostTimeStep;
    }
    return too_large_speed_flag ? cost : cost / kPointsNum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double TrajectoryPlanner::calcSafetyDistanceCost(const MatrixXd &otherCarState, const double &timeDuration,
                                                 const double &target_d)
{
    double cost = 0.;
    double t = 0.;
    const auto kPointNum = static_cast<size_t>(timeDuration / Helper::kCostTimeStep);
    const VectorXd kEgoCarCurrentState = evaluateStateAt(0);
    for (size_t i = 0; i < kPointNum; ++i)
    {
        const VectorXd kEgoCarState = evaluateStateAt(t);  //[s, s_dot, s_ddot, d, d_dot, d_ddot]
        // other car state is restore using kCostTimeStep gap
        if (Helper::distance(kEgoCarState(3), otherCarState(i, 1)) <= Helper::kMinimumLaterDis)
        {
            if (Helper::distance(otherCarState(i, 0), kEgoCarState(0)) <= Helper::kCarLength)
            {
                cost += Helper::kMaximumCost;
                ego_car_collision_points_.emplace_back(Point2D(kEgoCarState(0), kEgoCarState(3)));
                other_car_collision_points_.emplace_back(Point2D(otherCarState(i, 0), otherCarState(i, 1)));
            }
            if (otherCarState(0, 0) - kEgoCarCurrentState(0) > 0)
            {
                const double kRefDis = Helper::kLongitudinalDisGapIndex * kEgoCarState(1) + Helper::kLongitudinalDisGap;
                if (otherCarState(i, 0) - kEgoCarState(0) < kRefDis)
                {
                    cost += Helper::distance(kRefDis, Helper::distance(otherCarState(i, 0), kEgoCarState(0)));
                }
            }
            if (Helper::dToLaneNumber(target_d) !=
                Helper::dToLaneNumber(kEgoCarCurrentState(3)))  // lane changing consider cars at the back
            {
                if (otherCarState(0, 0) - kEgoCarCurrentState(0) < 0 &&
                    Helper::distance(otherCarState(i, 0), kEgoCarState(0)) <= 2.0 * Helper::kCarLength)  // at the back
                {
                    cost += Helper::kMaximumCost;
                    ego_car_collision_points_.emplace_back(Point2D(kEgoCarState(0), kEgoCarState(3)));
                    other_car_collision_points_.emplace_back(Point2D(otherCarState(i, 0), otherCarState(i, 1)));
                }
            }
        }
        t += Helper::kCostTimeStep;
    }
    return cost;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double TrajectoryPlanner::calcLaneOffsetCost(const double &timeDuration, const double &target_d)
{
    return 0.;  // as lane keeping and changing always target center of the lane
    double cost = 0;
    double t = 0.;
    const auto kPointsNum = static_cast<int>(timeDuration / Helper::kCostTimeStep);

    for (size_t i = 0; i < kPointsNum; i++)
    {
        const double d = calPolynominalDistanceAt(d_coeffs_, std::min(t, duration_d_));
        if (Helper::dToLaneNumber(d) != Helper::dToLaneNumber(target_d))  // lane changing do not count in this cost
        {
            return 0.;
        }
        cost += calcOnePointLaneOffsetCost(d);
        t += Helper::kCostTimeStep;
    }
    return cost / kPointsNum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double TrajectoryPlanner::calcOnePointLaneOffsetCost(const double &d)
{
    const int nLane = Helper::dToLaneNumber(d);
    if (nLane == -1) return Helper::kMaximumCost;
    const double laneCenter = Helper::laneNumberToD(nLane);
    const double distToCenter = Helper::distance(d, laneCenter);
    return distToCenter;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void TrajectoryPlanner::printInfo()
{
    const std::pair<double, double> &MinMaxV = minMaxVelocityS();
    std::cout << " startD: " << getStartD() << " end_d: " << getTargetD() << " C: " << getTotalCost()
              << " JC: " << getJerkCost() << " VC: " << getVelocityCost() << " DC: " << getSafetyDistanceCost()
              << " LC: " << getLaneOffsetCost() << " v_min: " << MinMaxV.first << " v_max: " << MinMaxV.second
              << std::endl;
}

void TrajectoryPlanner::printDetailedInfo()
{
    const auto kPointsNum = static_cast<int>(Helper::kPredictionTime / Helper::kCostTimeStep);
    double t = 0.;
    for (size_t i = 0; i < kPointsNum; i++)
    {
        VectorXd kEgoCarState = evaluateStateAt(t);  //[s, s_dot, s_ddot, d, d_dot, d_ddot]
        if (i % 10 == 0) std::cout << "trajectory s, d: " << kEgoCarState(0) << ", " << kEgoCarState(3) << std::endl;
        t += Helper::kCostTimeStep;
    }
}

void TrajectoryPlanner::printCollisionPointsInfo()
{
    for (size_t i = 0; i < ego_car_collision_points_.size(); i++)
    {
        std::cout << "ego_car collision_points s d: " << ego_car_collision_points_[i].x << ", "
                  << ego_car_collision_points_[i].y << std::endl;
        std::cout << "other car collision_points s d: " << other_car_collision_points_[i].x << ", "
                  << other_car_collision_points_[i].y << std::endl;
    }
}

void TrajectoryPlanner::clearCollisionPoints()
{
    ego_car_collision_points_.clear();
    other_car_collision_points_.clear();
}
}  // namespace autoparking_planning