/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  get waypoint_data and some processing

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once

#include <vector>
#include "common_define.h"
#include "helper.h"
#include "spline.h"
namespace autoparking_planning
{
class FrenetMap
{
   public:
    FrenetMap();
    ~FrenetMap();
    // transform sd to xy
    Eigen::Vector2d getXYFromFrenetSD(double s, double d) const;
    // Returns point [s, d] on the spline (Frenet-Map) which is the closest point to the (x,y).
    // s_start defines the search initial position on the spline.
    Eigen::Vector2d getFrenetSDFromXY(const Point2D& point_xy, double s_start) const;

   private:
    // Calculates derivative d/ds of error function [(x - x_spline)^2 + (y - y_spline)^2].
    double errorDerivative(const Point2D& point_xy, double s) const;
    // Returns normal to the spline at point s.
    Eigen::Vector2d getNormalAt(double s) const;
    void fitSpline();

   private:
    std::vector<WaypointData> waypoints_;
    tk::spline m_x_spline;  // x(s)
    tk::spline m_y_spline;  // y(s)
};
}  // end namespace