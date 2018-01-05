#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

#define POLY_ORDER  3
#define N_STATES    6

// These are the Hyper-Parameters of the MPC that needs to be tuned

// Set the time-step length and duration
const size_t N      = 30;          // maintaining long sight for the controller
const float  dt     = 0.075;

// This is the length from front to CoG that has a similar radius.
const double Lf    = 2.67;
const double ref_v = 100;   // 100 mph

// Setting Weights of the Objective Function
const float  W_cte       =   15.0;        //
const float  W_epsi      =   2.75;        //
const float  W_v_err     =   0.65;        // This like Kp in PID
const float  W_delta     =   50000.0;     // optimize your control
const float  W_delta_v   =   50.0;        // when you turn, lower your speed
const float  W_psi_des_v =   0.5;         // This plays the most dominant role in speed regulation
const float  W_acc       =   10.0;        //
const float  W_d_delta   =   150.0;       //
const float  W_d_acc     =   0.0;         // We want to keep change in acceleration free

// These are the indices of the "vars vector" and the "solution vector"
const size_t x_start     = 0;                   // 0*N;
const size_t y_start     = x_start     + N;     // 1*N;
const size_t psi_start   = y_start     + N;     // 2*N;
const size_t v_start     = psi_start   + N;     // 3*N;
const size_t cte_start   = v_start     + N;     // 4*N;
const size_t epsi_start  = cte_start   + N;     // 5*N;
const size_t delta_start = epsi_start  + N;     // 6*N;
const size_t acc_start   = delta_start + N - 1; // 7*N - 1;


class MPC {
 public:

  vector<double> x_predict;
  vector<double> y_predict;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
