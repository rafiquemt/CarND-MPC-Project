#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
const double Lf = 2.67;
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char** argv) {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  initParams(argv);
  const double delay_estimate_s = 0.150; // starting delay for 100ms sleep + simulator delay
  double lastDelayEstimate = delay_estimate_s;

  h.onMessage([&mpc, &lastDelayEstimate](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    auto start = std::chrono::system_clock::now();
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"]; // vehicle coordinates in global map
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v = v * 0.44704; // convert from mph to m/s
          double delta = j[1]["steering_angle"];
          delta = delta * -1; 
          
          Eigen::VectorXd ptsx_local(ptsx.size());
          Eigen::VectorXd ptsy_local(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            double xi = ptsx[i] - px;
            double yi = ptsy[i] - py;
            ptsx_local(i) = (xi * cos(-psi) - yi * sin(-psi));
            ptsy_local(i) = (xi * sin(-psi) + yi * cos(-psi));
          }
          auto coeffs = polyfit(ptsx_local, ptsy_local, 3);
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
          double v0 = v;
          double cte0 = polyeval(coeffs, x0);
          double epsi0 = psi0 - atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * x0 * x0));
          
          // transform state forward in time to deal with delay
          x0 = x0 + v0 * lastDelayEstimate;
          y0 = y0 + 0;
          psi0 = psi0 + (v0/Lf) * delta * lastDelayEstimate;
          v0 = v0 + 0; // no change assuming 0 acceleration (approx.)
          cte0 = cte0 + v0 * sin(epsi0) * lastDelayEstimate;
          epsi0 = epsi0 + (v0/Lf) * delta * lastDelayEstimate;

          Eigen::VectorXd state(6);
          state << x0, y0, psi0, v0, cte0, epsi0;

          double steer_value = 0;
          double throttle_value = 0;

          json msgJson;
          cout << "About to solve";
          auto result = mpc.Solve(state, coeffs);
          
          steer_value = ((result[0] * -1) / deg2rad(25)); // flip because simulator uses reverse values
          throttle_value = result[1];
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 2; i < result.size(); i++) {
            if (i % 2 == 0) {
              mpc_x_vals.push_back(result[i]);
            } else {
              mpc_y_vals.push_back(result[i]);
            }
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 0; i < num_points; i++) {
            next_x_vals.push_back(i * poly_inc);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> diff = end - start;
          lastDelayEstimate = diff.count();
          cout << "Delay Estimate: " << lastDelayEstimate << endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
