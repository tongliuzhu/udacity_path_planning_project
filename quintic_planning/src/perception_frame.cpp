#include <algorithm>
#include <iostream>
#include <vector>
#include "frenet_map.h"
#include "perception_frame.h"

namespace autoparking_planning
{
PerceptionFrame::PerceptionFrame()
{
    perceived_cars_.clear();
    sensor_fusion_data_.clear();
}

PerceptionFrame::~PerceptionFrame() {}
void PerceptionFrame::setSensorFusionData(std::vector<SensorFusionData>& sensor_fusion_data,
                                          const FrenetMap& frenet_map)
{
    sensor_fusion_data_.resize(sensor_fusion_data.size());
    for (size_t i = 0; i < sensor_fusion_data.size(); ++i)
    {
        sensor_fusion_data_[i].id = sensor_fusion_data[i].id;
        sensor_fusion_data_[i].x = sensor_fusion_data[i].x;
        sensor_fusion_data_[i].y = sensor_fusion_data[i].y;
        sensor_fusion_data_[i].vx = sensor_fusion_data[i].vx;
        sensor_fusion_data_[i].vy = sensor_fusion_data[i].vy;
        sensor_fusion_data_[i].s = sensor_fusion_data[i].s;
        sensor_fusion_data_[i].d = sensor_fusion_data[i].d;
    }
    updatePerceivedCars(frenet_map);
}

void PerceptionFrame::predictFutureFusionData(double delta_time)
{
    for (auto& elem : perceived_cars_)
    {
        elem.second.predictOtherCarState(delta_time);
    }
}

std::vector<Eigen::MatrixXd> PerceptionFrame::getLeadingCarsTrajectoryInLane(const Eigen::VectorXd& ego_car_state_v6,
                                                                             int lane_num, double time_duration) const
{
    std::vector<Eigen::MatrixXd> leading_cars_trajectories;
    auto other_cars = getLeadingCarsInLane(ego_car_state_v6, lane_num);
    for (OtherCar& car : other_cars)
    {
        leading_cars_trajectories.emplace_back(car.predictedTrajectory(time_duration));
    }
    return leading_cars_trajectories;
}

std::vector<Eigen::MatrixXd> PerceptionFrame::getOtherCarsTrajectoryInLane(const Eigen::VectorXd& ego_car_state_v6,
                                                                           int lane_num, double time_duration) const
{
    std::vector<Eigen::MatrixXd> other_car_trajectories;
    std::vector<OtherCar> OtherCars = getNearestCarsInLane(ego_car_state_v6, lane_num);
    for (OtherCar& car : OtherCars)
    {
        other_car_trajectories.emplace_back(car.predictedTrajectory(time_duration));
    }
    return other_car_trajectories;
}

std::vector<OtherCar> PerceptionFrame::getLeadingCarsInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                                            bool is_only_nearest) const
{
    std::vector<OtherCar> leading_cars;
    const auto s = ego_car_state_v6(0);
    for (const auto& elem : perceived_cars_)
    {
        const auto& car = elem.second;
        if (car.getS() > s && car.isInlane(lane_num))
        {
            leading_cars.emplace_back(car);
        }
    }
    // sorts by ascending S
    sort(leading_cars.begin(), leading_cars.end(),
         [](const OtherCar& l, const OtherCar& r) { return l.getS() < r.getS(); });
    if (is_only_nearest && leading_cars.size() > 0)
    {
        leading_cars.resize(1);
        // std::cout << "leading car in lane: " << (leading_cars[0].getCarState())(0)
        // << " ego car: " << ego_car_state_v6(0) << " dis: " <<
        // (leading_cars[0].getCarState())(0) - ego_car_state_v6(0) << std::endl;
    }
    return leading_cars;
}

std::vector<OtherCar> PerceptionFrame::getNearestCarsInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                                            double delta_s) const
{
    std::vector<OtherCar> nearest_cars;
    for (const auto& elem : perceived_cars_)
    {
        const auto& car = elem.second;
        if (car.isInlane(lane_num) && Helper::distance(car.getS(), ego_car_state_v6(0)) <= delta_s)
        {
            nearest_cars.emplace_back(car);
        }
    }
    return nearest_cars;
}

void PerceptionFrame::updatePerceivedCars(const FrenetMap& frenet_map)
{
    // TODO optimize vx vy to sd dd, currently all dd set to 0.
    perceived_cars_.clear();
    for (auto& sf : sensor_fusion_data_)
    {
        const auto kVelocity = sqrt(sf.vx * sf.vx + sf.vy * sf.vy);
        const Eigen::Vector2d kSDValue = frenet_map.getFrenetSDFromXY(Point2D(sf.x, sf.y), sf.s);
        perceived_cars_[sf.id] = OtherCar(kSDValue(0), kSDValue(1), kVelocity, 0.);
    }
}
}  // End namespace