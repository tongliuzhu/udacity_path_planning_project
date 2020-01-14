#include "other_car.h"
namespace autoparking_planning
{
OtherCar::OtherCar(double s, double d, double vs, double vd)
{
    car_state_ = Eigen::VectorXd::Zero(4);
    car_state_ << s, d, vs, vd;
}

OtherCar::~OtherCar() {}
Eigen::MatrixXd OtherCar::predictedTrajectory(const double& duration)
{
    const auto kSizeNum = static_cast<size_t>(duration / Helper::kPredictionTimeStep);
    Eigen::MatrixXd result(kSizeNum, 4);

    double s = car_state_(0);  //[s, d, vs, vd]
    double d = car_state_(1);

    for (size_t i = 0; i < kSizeNum; ++i)
    {
        result.row(i) << s, d, car_state_(2), car_state_(3);
        s += car_state_(2) * Helper::kPredictionTimeStep;
        d += car_state_(3) * Helper::kPredictionTimeStep;
    }
    return result;
}
}  // end namespace