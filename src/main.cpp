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
using Eigen::VectorXd;

// Define the iterations counter
int32_t i_count = 0;
double Sum_cost = 0.0;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  static VectorXd prev_coeffs(POLY_ORDER+1);
  prev_coeffs.fill(0.0);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          ///////////////////////////////////////

          VectorXd state(N_STATES);
          VectorXd coeffs(POLY_ORDER+1);
          VectorXd coeffs_raw(POLY_ORDER+1);

          double steer_value;
          double throttle_value;

          const size_t N_X_PTS = ptsx.size();
          const size_t N_Y_PTS = ptsy.size();

          // N_X_PTS must equal N_Y_PTS
          assert(N_X_PTS == N_Y_PTS);

          // Calculate the waypoints in Vehicle coordinates
          VectorXd V_ptsX(N_X_PTS);
          VectorXd V_ptsY(N_Y_PTS);

          const double sin_psi = sin(-psi);
          const double cos_psi = cos(-psi);

          for (size_t i=0; i< N_X_PTS; i++)
            V_ptsX[i] = (ptsx[i] - px) * cos_psi - (ptsy[i] - py) * sin_psi;
          for (size_t i=0; i< N_Y_PTS; i++)
            V_ptsY[i] = (ptsy[i] - py) * cos_psi + (ptsx[i] - px) * sin_psi;

          //std::cout << " ptsX0: " << ptsX(0) << " ptsY5: " << ptsY[5] << std::endl;

          // Calculate the Polynomial Coefficients (3rd Order)
          // From the received Data (X, Y)

          coeffs_raw = polyfit(V_ptsX, V_ptsY, POLY_ORDER);

          for(size_t i=0; i < (POLY_ORDER+1); i++)
          {
              const float K_coeff = 0.9;
              coeffs[i] = K_coeff*coeffs_raw[i] + (1-K_coeff)*prev_coeffs[i];
          }
          prev_coeffs = coeffs_raw;

          //std::cout << " coeff0: " << coeffs[0] << " coeff1: " << coeffs[1] << std::endl;

          //double cte        = polyeval(coeffs, px) - py;
          double cte   = coeffs[0];  // In Vehicle Coordinates - since px = 0, py =0
          double epsi  = - atan(coeffs[1]);   // In Vehicle Coordinates - since px = 0, py =0

          state[0] = 0.0;   // In Vehicle Coordinates px  = 0.0
          state[1] = 0.0;   // In Vehicle Coordinates py  = 0.0
          state[2] = 0.0;   // In Vehicle Coordinates psi = 0.0
          state[3] = v;
          state[4] = cte;
          state[5] = epsi;

          //std::cout << " state[0] " << state[0] << " state[1] " << state[1] << std::endl;

          vector<double> control_action = mpc.Solve(state, coeffs);

          steer_value    = control_action[0] / 0.436332; // Delta: Divide by (25 Degree in radians )to let the limits between [-1 & 1].;
          throttle_value = control_action[1];            // Acceleration

          // Develop a Performance Indicator
          #define CYCLE_THRESHOLD  300
          double Perf_Ind = 0.0;
          //Sum_cost += control_action[2];
          Sum_cost += cte*cte + 100 * epsi * epsi;
          // Increment counter
          i_count++;
          if (i_count > CYCLE_THRESHOLD)
            {
                Perf_Ind = Sum_cost / i_count;
                Sum_cost = 0.0;
                i_count  = 0;

                cout<<"Perf_Ind: "<<Perf_Ind << endl;
            }
          //cout<<"Count: "<<i_count<<" steer_value "<<steer_value<<" throttle_value "<<throttle_value<<endl;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"]       = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (size_t i=0 ; i < N; i++)
          {
            mpc_x_vals.push_back(mpc.x_predict[i]);
            mpc_y_vals.push_back(mpc.y_predict[i]);
          }

          // mpc_x_vals.push_back(output_result[delta_start])

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (size_t j=0 ; j < N_X_PTS; j++)
          {
            next_x_vals.push_back(V_ptsX[j]);
            next_y_vals.push_back(V_ptsY[j]);
          }

/*          //double xx = 0;
          for (double xx = 0.0 ; xx < 100.0; xx +=5.0)
          {
            next_x_vals.push_back(xx);
            next_y_vals.push_back(polyeval(coeffs, xx));
          }
*/
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
