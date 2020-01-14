#include "helper.h"

namespace Helper
{
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string input_string)
{
    auto found_null = input_string.find("null");
    auto b1 = input_string.find_first_of("[");
    auto b2 = input_string.find_first_of("}");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return input_string.substr(b1, b2 - b1 + 2);
    }
    else
    {
        return "";
    }
}

// For converting back and forth between radians and degrees.
double deg2Rad(const double x) { return x * M_PI / 180.; }
double rad2Deg(const double x) { return x * 180. / M_PI; }
// Calculate distance of two points

double distance(const double& x1, const double& y1, const double& x2, const double& y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Returns lane number -1, 0, 1, 2 for specified d coord. in Frenet frame.
int dToLaneNumber(const double& d)
{
    if (d >= 0. || d <= -12.) return -1;  // outside of right-driving road.
    if (d <= -8)
        return 0;
    else if (d <= -4)
        return 1;
    else
        return 2;
}

// Returns D - center of Lane for specified Lane number
double laneNumberToD(int nLaneNumber)
{
    const std::array<double, 3> laneCenter = {-9.8, -6.0, -2.0};  // try to move in the road
    assert(nLaneNumber >= 0 && nLaneNumber < 3);
    return laneCenter[nLaneNumber];
}
}  // end namespace
