#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "frenet_map.h"

namespace autoparking_planning
{
FrenetMap::FrenetMap()
{
    waypoints_.clear();
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    const std::string map_file_ = "../data/highway_map.csv";
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
    // Read data into waypoints_
    WaypointData temp_waypoint;
    std::string line;
    const size_t kMapDataSize = 200;
    waypoints_.reserve(kMapDataSize);
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> temp_waypoint.x;
        iss >> temp_waypoint.y;
        iss >> temp_waypoint.s;
        iss >> temp_waypoint.d_x;
        iss >> temp_waypoint.d_y;
        waypoints_.emplace_back(temp_waypoint);
    }
    fitSpline();
}

FrenetMap::~FrenetMap() {}
Eigen::Vector2d FrenetMap::getXYFromFrenetSD(double s, double d) const
{
    Helper::normalizeS(s);
    Eigen::Vector2d pt(m_x_spline(s), m_y_spline(s));
    // normal vector
    Eigen::Vector2d nv(-m_y_spline.deriv(1, s), m_x_spline.deriv(1, s));
    return (pt + nv * d);
}

Eigen::Vector2d FrenetMap::getFrenetSDFromXY(const Point2D &point_xy, double s_start) const
{
    // Gradient descent is used to find the point (s,d) on the spline, which is closest to point
    // point_xy.
    const double eps = 1.0e-6;
    const double gamma = 0.001;
    const double precision = 1e-12;
    double s = s_start;
    double prev_step_size = s;

    while (prev_step_size > precision)
    {
        const auto prev_s = s;
        s = s - gamma * errorDerivative(point_xy, prev_s);
        prev_step_size = std::abs(s - prev_s);
    }

    Eigen::Vector2d temp_point(point_xy.x, point_xy.y);
    const Eigen::Vector2d p_spline(m_x_spline(s), m_y_spline(s));
    const Eigen::Vector2d p_delta = (temp_point - p_spline).array() / getNormalAt(s).array();
    const double d = 0.5 * (p_delta(0) + p_delta(1));
    // const double d = -sqrt(pow((point_xy.x -m_x_spline(s)), 2) + pow(point_xy.y -m_y_spline(s),
    // 2));
    return Eigen::Vector2d(s, d);
}

double FrenetMap::errorDerivative(const Point2D &point_xy, double s) const
{
    return -2. * (point_xy.x - m_x_spline(s)) * m_x_spline.deriv(1, s) -
           2. * (point_xy.y - m_y_spline(s)) * m_y_spline.deriv(1, s);
}

Eigen::Vector2d FrenetMap::getNormalAt(double s) const
{
    return Eigen::Vector2d(-m_y_spline.deriv(1, s), m_x_spline.deriv(1, s));
}

void FrenetMap::fitSpline()
{
#if 1
    //------------------------------------
    // Fix of kink at the end of the track!
    WaypointData wp_front = waypoints_.front();
    WaypointData wp_back = waypoints_.back();

    WaypointData wp1 = wp_back;
    wp1.s -= Helper::kMaxSVlaue;

    WaypointData wp2 = wp_front;
    wp2.s += Helper::kMaxSVlaue;

    waypoints_.insert(waypoints_.begin(), wp1);
    waypoints_.emplace_back(wp2);
//------------------------------------
#endif
    std::vector<double> x, y, s;
    x.resize(waypoints_.size());
    y.resize(waypoints_.size());
    s.resize(waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); i++)
    {
        x[i] = waypoints_[i].x;
        y[i] = waypoints_[i].y;
        s[i] = waypoints_[i].s;
    }
    m_x_spline.set_points(s, x);
    m_y_spline.set_points(s, y);
}
}  // namespace autoparking_planning