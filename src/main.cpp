#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // The car drives well around the track with this initial parameter of 1, 0, 0. at throttle .1

  // followed wikipedia tuning section, raised Kp from 1 to 2 to get oscialliton. Then tried values for Kd starting at 1 which caused overshooting and lowered till very small values started working. Then tried Ki values from .5 up to correct overshooting. settled on the values below.

  // pid.Init(1, 0, 0);
  pid.Init(1, .001, 2);

  PID speedPid;
  // speedPid.Init(.125, .0001, 0.797906);
  speedPid.Init(1, 0, 0);

  bool use_speed_pid;
  use_speed_pid = false;

  double target_speed;
  target_speed = 0.3;

  h.onMessage([&pid, &speedPid, &target_speed, &use_speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          double speed_value;
          double speed_cte;

          // std::cout << "FROM SIM, CTE: " << cte << " Speed: " << speed << " Angle: " << angle << " in radians: " << steer_value << std::endl;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);

          steer_value = -pid.GetOutput();

          if (steer_value < -1) {
            steer_value = -1;
          }

          if (steer_value > 1) {
            steer_value = 1;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          if (use_speed_pid) {

            speed_cte = speed - target_speed;

            speedPid.UpdateError(speed_cte);
            speed_value = speedPid.GetOutput();

            std::cout << "Speed CTE: " << speed_cte << " Speed Value: " << speed_value << std::endl;

            if (speed_value < .1) {
              speed_value = .1;
            }

            if (speed_value > .2) {
              speed_value = .2;
            }
          } else {

            // slow down when steering
            if (steer_value > .4 || steer_value < -.4) {
              speed_value = 0.1;
            } else if (steer_value > .2 || steer_value < -.2) {
              speed_value = 0.15;
            } else {
              speed_value = 0.2;
            }

            // hardcode to slow single speed for initial parameter tuning
            // speed_value = .1;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

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
