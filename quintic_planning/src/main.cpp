#include <math.h>
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <libgen.h>
#include <unistd.h>

#include "Eigen/Core"
#include "Eigen/Eigen"

#include "common_define.h"
#include "ego_car.h"
#include "frenet_map.h"
#include "helper.h"
#include "json.h"
#include "spline.h"

using namespace std;
using namespace autoparking_planning;

// for convenience
using json = nlohmann::json;

int main(int argc, char *argv[])
{
    chdir(dirname(argv[0]));  // set path of the .exe as current path dir
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    FrenetMap frenet_map;
    EgoCar ego_car(frenet_map);

    h.onMessage([&ego_car](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        // auto sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        // timeval start, end;
        // gettimeofday(&start, 0);
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = Helper::hasData(data);
            if (s != "")
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry")
                {
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];

                    // j[1] is the data JSON object
                    // Main car's localization Data
                    CarLocalizationData car_pose;
                    car_pose.x = j[1]["x"];
                    car_pose.y = j[1]["y"];
                    car_pose.s = j[1]["s"];
                    car_pose.d = j[1]["d"];
                    car_pose.yaw = j[1]["yaw"];
                    car_pose.speed = j[1]["speed"];
                    car_pose.d = -car_pose.d;  // bug fix

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    std::vector<SensorFusionData> sensor_fusion_data;
                    sensor_fusion_data.resize(sensor_fusion.size());
                    for (size_t i = 0; i < sensor_fusion.size(); ++i)
                    {
                        sensor_fusion_data[i].id = sensor_fusion[i][0];
                        sensor_fusion_data[i].x = sensor_fusion[i][1];
                        sensor_fusion_data[i].y = sensor_fusion[i][2];
                        sensor_fusion_data[i].vx = sensor_fusion[i][3];
                        sensor_fusion_data[i].vy = sensor_fusion[i][4];
                        sensor_fusion_data[i].s = sensor_fusion[i][5];
                        sensor_fusion_data[i].d = sensor_fusion[i][6];
                    }

                    ego_car.updateTrajectory(car_pose, sensor_fusion_data, previous_path_x, previous_path_y);
                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every
                    // .02 seconds
                    json msgJson;
                    msgJson["next_x"] = ego_car.get_next_x_vals();
                    msgJson["next_y"] = ego_car.get_next_y_vals();
                    auto msg = "42[\"control\"," + msgJson.dump() + "]";
                    // this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection(
        [&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}