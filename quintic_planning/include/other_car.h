/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  OtherCar class for all other vehicles' data and processing

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include "helper.h"
namespace autoparking_planning
{
class OtherCar
{
   public:
    OtherCar(double s = 0., double d = 0., double vs = 0., double vd = 0.);
    ~OtherCar();

    inline int getLaneNum() const { return Helper::dToLaneNumber(getD()); }
    inline bool isInlane(int lane) const { return lane == getLaneNum() && lane >= 0; }
    inline double getS() const { return car_state_(0); }
    inline double getD() const { return car_state_(1); }
    inline double getV() const { return car_state_(2); }
    inline Eigen::VectorXd getCarState() const { return car_state_; }
    inline void predictOtherCarState(const double& deltaTime)
    {
        car_state_(0) += car_state_(2) * deltaTime;
        car_state_(1) += car_state_(3) * deltaTime;
    }
    inline void updateOtherCarState(const double& s, const double& d, const double& vs, const double& vd)
    {
        car_state_ << s, d, vs, vd;
    }

    // Generates trajectory as matrix of rows [s, d, vs, vd].
    // Number of rows = duration / delataTime.
    Eigen::MatrixXd predictedTrajectory(const double& duration);

   private:
    Eigen::VectorXd car_state_;  //[s, d, vs, vd]
};
}  // end namespace