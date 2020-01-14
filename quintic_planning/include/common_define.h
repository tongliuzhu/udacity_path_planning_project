/*
 \author  Tong Liuzhu, tongliuzhu@126.com

  \brief  some commonly used data structure and constants

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include <cassert>

#include "Eigen/Core"
#include "Eigen/Eigen"
namespace autoparking_planning
{
struct Point2D
{
    double x;
    double y;
    // default constructor
    Point2D(double x1 = 0, double y1 = 0) : x(x1), y(y1) {}
    //  overload *
    Point2D& operator*(double& index)
    {
        this->x *= index;
        this->y *= index;
        return *this;
    }
    // overload +
    Point2D& operator+(Point2D& add_point)
    {
        this->x += add_point.x;
        this->y += add_point.y;
        return *this;
    }
    // overload ()
    double& operator()(int index)
    {
        assert(index >= 0 && index < 2);
        if (0 == index)
            return this->x;
        else
            return this->y;
    }
};

struct WaypointData
{
    double x;
    double y;
    double s;
    double d_x;
    double d_y;

    WaypointData(double x = 0, double y = 0, double s = 0, double d_x = 0, double d_y = 0)
        : x(x), y(y), s(s), d_x(d_x), d_y(d_y)
    {
    }
};

struct CarLocalizationData
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    CarLocalizationData(double x = 0, double y = 0, double s = 0, double d = 0, double yaw = 0, double speed = 0)
        : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed)
    {
    }
};

struct SensorFusionData
{
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    SensorFusionData(int x = 0, double y = 0, double vx = 0, double vy = 0, double s = 0, double d = 0)
        : x(x), y(y), vx(vx), vy(vy), s(s), d(d)
    {
    }
};
}
