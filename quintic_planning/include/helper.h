/*
\author  Tong Liuzhu, tongliuzhu@126.com

  \brief  some commonly used helper functions

  \version 1.0
  \date Oct 20th, 2019
*/
#pragma once
#include <assert.h>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Eigen"

namespace Helper
{
// some configurations TODO into config files
const double kTrajectoryTimeStep = 0.02;  // 20ms between waypoints
const double kLaneWidth = 4.;

const double kPredictionTime = 9.0;
const double kChangeLaneTime = 4.0;
const double kCostTimeStep = 0.1;        // evaluation
const double kPredictionTimeStep = 0.1;  // both for our and other car should be equal to kCostTimeStep

const double kSpeedLimit = 46.5 * 0.44704;    // mps
const double kMaximumJerk = 9.;               // m/s^3
const double kMaximumAcc = 8.;                // m/s^2
const double kMinimumLaterDis = 3.0;          // Minimum lateral dis betweeen egocar and target car
const double kLongitudinalDisGap = 8.;        // kLongitudinalDisGap + k*v
const double kLongitudinalDisGapIndex = 1.1;  // kLongitudinalDisGap + k*v
const double kCarLength = 6.;                 // 6m

const double kMaximumCost = 10000.;  //

const int kMaximumLaneNum = 2;
const int kMinimumLaneNum = 0;

// The max s value before wrapping around the track back to 0
const double kMaxSVlaue = 6945.554;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string input_string);

// For converting back and forth between radians and degrees.
double deg2Rad(const double x);
double rad2Deg(const double x);

// Calculate distance of two points
double distance(const double& x1, const double& y1, const double& x2, const double& y2);

template <class T>
T distance(T x1, T x2)
{
    return std::abs(x2 - x1);
}

// Returns lane number -1, 0, 1, 2 for specified d coord. in Frenet frame.
int dToLaneNumber(const double& d);

// Returns D - center of Lane for specified Lane number
double laneNumberToD(int nLaneNumber);

// time evaluation of functions
struct FunctionTimer
{
    std::chrono::high_resolution_clock::time_point t2, t1;
    std::string function_name;
    FunctionTimer(std::string name) : function_name(name)
    {
        t1 = std::chrono::high_resolution_clock::now();
        t2 = std::chrono::high_resolution_clock::now();
    }

    ~FunctionTimer()
    {
        t2 = std::chrono::high_resolution_clock::now();
        std::cout << function_name << " takes " << std::chrono::duration<double, std::milli>(t2 - t1).count()
                  << " ms\n";
    }
};

inline void normalizeS(double& s)
{
    (s > kMaxSVlaue) ? (s -= kMaxSVlaue) : s;
    (s < -kMaxSVlaue) ? (s += kMaxSVlaue) : s;
}
}  // end namespace
