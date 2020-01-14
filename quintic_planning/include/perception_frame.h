/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  perception_frame class for all the perception data and processing

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include <unordered_map>
#include "common_define.h"
#include "other_car.h"

namespace autoparking_planning
{
class FrenetMap;

class PerceptionFrame
{
   public:
    PerceptionFrame();
    ~PerceptionFrame();
    void predictFutureFusionData(double delta_time);
    std::vector<SensorFusionData>& SensorFusionStorage() { return sensor_fusion_data_; }
    void setSensorFusionData(std::vector<SensorFusionData>& sensor_fusion_data, const FrenetMap& frenet_map);

    inline std::vector<SensorFusionData> getSensorFusionData() { return sensor_fusion_data_; };
    // Returns all other leading vehicles in the lane which is defined by vehicle state [s, s_d, s_dd,
    // d, d_d, d_dd].
    std::vector<Eigen::MatrixXd> getLeadingCarsTrajectoryInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                                                double time_duration) const;

    // Returns all vehicles in the lane which are close to car's S-position: [StateV6(0)-deltaS,
    // StateV6(0)+deltaS].
    std::vector<Eigen::MatrixXd> getOtherCarsTrajectoryInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                                              double time_duration) const;

    // Returns other leading vehicles in the lane which is defined by vehicle state [s, s_d, s_dd, d,
    // d_d, d_dd].
    std::vector<OtherCar> getLeadingCarsInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                               bool is_only_nearest = true) const;

    // Returns all vehicles in the lane which are close to car's S-position: [sdcStateV6(0)-deltaS,
    // sdcStateV6(0)+deltaS].
    std::vector<OtherCar> getNearestCarsInLane(const Eigen::VectorXd& ego_car_state_v6, int lane_num,
                                               double delta_s = 200) const;

   private:
    // Update all the data (only cars here) into perceived cars
    void updatePerceivedCars(const FrenetMap& frenet_map);

   private:
    std::unordered_map<int, OtherCar> perceived_cars_;
    std::vector<SensorFusionData> sensor_fusion_data_;
};
}  // end namespace