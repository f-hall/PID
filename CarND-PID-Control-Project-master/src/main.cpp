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

// if 'true' use twiddle on track, if false just the init-values will be used.
bool twiddle_switch = false;

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
  PID pid_speed;
  // I tried a lot of different values for steering and speed manually.
  // Then I used twiddle to get even better results which resulted in these init values.
  pid.Init(0.0772694, 0.0075422, 0.960661);
  pid_speed.Init(0.125 , 0.0001 , 0.8);

  h.onMessage([&pid, &pid_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          // Using the UpdateError functions for both (speed and steering) to get the values
          pid.UpdateError(cte);
          double steer_value = pid.getValue();
          double new_speed = 30.*(1.-abs(steer_value)) + 20.;
          pid_speed.UpdateError(speed - new_speed);

          // You can use these lines to look up the current values and errors.
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "Speed-Error: " << speed-new_speed << " Speed Value: " << pid_speed.getValue_speed() << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = pid_speed.getValue_speed();
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Using twiddle (if true) to test different values for speed and steering
          if(twiddle_switch == true && (pid.getNumberOfSteps() > 400))
          {
              pid.twiddle();
              // you can use the following line to get the - with twiddle updated - values
              std::cout << pid.getKp() << ", " << pid.getKi() << ", " << pid.getKd() << std::endl;
          }
          if(twiddle_switch == true && (pid_speed.getNumberOfSteps() > 400))
          {
              pid_speed.twiddle();
              // you can use the following line to get the - with twiddle updated - values
              std::cout << pid_speed.getKp() << ", " << pid_speed.getKi() << ", " << pid_speed.getKd() << std::endl;
          }
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
    	  if(twiddle_switch == true) {
    		  std::string msg = "42[\"manual\",{}]";
    		  if(twiddle_switch == 0 && pid.getNumberOfSteps() > 400){
    			  pid.twiddle();
    			  pid.setNumberOfSteps(0);
    			  pid_speed.twiddle();
    			  pid_speed.setNumberOfSteps(0);
    		  }
    		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    	  } else {
    		  std::string msg = "42[\"manual\",{}]";
    		  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    	  }
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
