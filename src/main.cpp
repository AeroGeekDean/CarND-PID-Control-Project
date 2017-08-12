#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <chrono>

// for convenience
using json = nlohmann::json;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// For keeping track of time to compute dt
steady_clock::time_point time_past;
duration<double> delta_t;

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

  PID steering_controller;
  PID cruise_controller;

  // TODO: Initialize the pid variable.
  double steer_kp = 1.0;
  double steer_ki = 0.1;
  double steer_kd = 0.5;

  double cruise_kp = 0.1;
  double cruise_ki = 0.02; // kp*frame_rate
  double cruise_kd = 0.0;

  steering_controller.Init(steer_kp, steer_ki, steer_kd);
  steering_controller.SetLimits(1.0, 0.25);

  cruise_controller.Init(cruise_kp, cruise_ki, cruise_kd);
  cruise_controller.SetLimits(1.0, 0.5);

  static bool sim_initialized = false;

  h.onMessage([&steering_controller, &cruise_controller](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>()); // [m], + right
          double speed = std::stod(j[1]["speed"].get<std::string>()); // [mph]
          double steer_fb = std::stod(j[1]["steering_angle"].get<std::string>()); // [deg], (-25, +25)
          double steer_value; // [N/D] (-1, +1)
          double throttle; // [N/D] (-1, +1)
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          if (!sim_initialized)
          {
            time_past = steady_clock::now();
            sim_initialized = true;
          }

          // Compute elapsed time since last frame
          steady_clock::time_point time_now = steady_clock::now();
          delta_t = duration_cast<duration<double>>( time_now - time_past ); // dt in [sec]
          double dt = delta_t.count();
          time_past = time_now; // update past value

          // Compute steering
          steering_controller.UpdateError(dt, cte);
          steer_value = steering_controller.TotalError();
//          steer_value = 0.0;

          // Computer throttle
          double cruise_set_spd = 20.0;
          cruise_controller.UpdateError(dt, -(cruise_set_spd - speed));
          throttle = cruise_controller.TotalError();

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Hz " << 1./dt << "   |    Speed: " << speed << " Throttle: " << throttle
                    << " CTE: " << cte << " Steering: " << steer_value
                    << ((steering_controller.isIntegratorSaturated) ? " i-Saturated!!" : "")
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
//          msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
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
