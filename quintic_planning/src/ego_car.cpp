#include <sys/time.h>
#include <unistd.h>
#include <ctime>
#include <iostream>
#include <memory>
#include "behavior_planner.h"
#include "frenet_map.h"
#include "helper.h"
#include "lane_keeping.h"
#include "perception_frame.h"
#include "ego_car.h"

namespace autoparking_planning
{
EgoCar::EgoCar(const FrenetMap& frenet_map) : frenet_map_(frenet_map), current_frenet_state_(Eigen::VectorXd::Zero(6))
{
    next_x_pos_.clear();
    next_y_pos_.clear();
}

EgoCar::~EgoCar() {}
void EgoCar::updateTrajectory(CarLocalizationData& current_state, std::vector<SensorFusionData>& sensor_fusion_data,
                              const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y)
{
    // Helper::FunctionTimer functionTimer("updateTrajectory");
    std::chrono::high_resolution_clock::time_point t2, t1;
    t1 = std::chrono::high_resolution_clock::now();
    const int kPredictionPathSize = int(prediction_trajectory_time_ / Helper::kTrajectoryTimeStep);
    const int kReactivePathSize = int(reactive__trajectory_time_ / Helper::kTrajectoryTimeStep);
    // get current frenet state
    const double kMph2Mps = 0.44704;
    current_state.speed = current_state.speed * kMph2Mps;

    next_x_pos_.clear();
    next_y_pos_.clear();
    next_x_pos_.reserve(kPredictionPathSize);
    next_y_pos_.reserve(kPredictionPathSize);
    if (previous_path_x.size() < kReactivePathSize)  // if first time or no enough point left
    {
        Eigen::VectorXd temp_current_state = Eigen::VectorXd::Zero(6);
        Point2D temp_point2d(current_state.x, current_state.y);
        const Eigen::Vector2d& temp_frenet = frenet_map_.getFrenetSDFromXY(temp_point2d, current_state.s);
        temp_current_state << temp_frenet(0), current_state.speed, 0., temp_frenet(1), 0.0, 0.;
        setCurrentState(temp_current_state);
    }
    const int kPreviousPathSize = static_cast<int>(previous_path_x.size());
    const int kPreviousPredictionPathSize = std::max(0, kPreviousPathSize - (kPredictionPathSize - kReactivePathSize));

    for (size_t i = 0; i < kPreviousPredictionPathSize; ++i)
    {
        next_x_pos_.emplace_back(previous_path_x[i]);
        next_y_pos_.emplace_back(previous_path_y[i]);
    }
    const double kDelayTime = kPreviousPredictionPathSize * Helper::kTrajectoryTimeStep;

    setPerceptionFrameData(sensor_fusion_data);
    perception_frame_.predictFutureFusionData(kDelayTime);
    TrajectorySharedPtr p_trajectory = ego_behavior_planner_.behaviorPlanning(current_frenet_state_, perception_frame_);
    Eigen::VectorXd temp_state = current_frenet_state_;
    for (size_t index = 0; index < kPredictionPathSize - kPreviousPredictionPathSize; ++index)
    {
        if (index == kReactivePathSize - kPreviousPredictionPathSize)
        {
            current_frenet_state_ = temp_state;
        }
        const Eigen::Vector2d next_point = frenet_map_.getXYFromFrenetSD(temp_state(0), temp_state(3));
        assert(nullptr != p_trajectory);
        temp_state = p_trajectory->evaluateStateAt(double(index + 1) * (Helper::kTrajectoryTimeStep));  // get sd
        Helper::normalizeS(temp_state(0));
        next_x_pos_.emplace_back(next_point(0));
        next_y_pos_.emplace_back(next_point(1));
    }
    t2 = std::chrono::high_resolution_clock::now();
    static double timer = 0.;
    static int num = 0;
    num++;
    timer += std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "loops: " << num << " takes time: " << timer << std::endl;
    // exit(0);
}

void EgoCar::setPerceptionFrameData(std::vector<SensorFusionData>& sensor_fusion_data)
{
    perception_frame_.setSensorFusionData(sensor_fusion_data, frenet_map_);
}
}  // End namespace
