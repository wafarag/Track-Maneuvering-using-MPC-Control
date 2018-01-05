#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//

double polyeval(Eigen::VectorXd coeffs, double x);

class FG_eval
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars)
  {
    //  Implement the MPC objective function
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Setup the rest of the model constraints
    fg[1 + x_start    ] = vars[x_start];
    fg[1 + y_start    ] = vars[y_start];
    fg[1 + psi_start  ] = vars[psi_start];
    fg[1 + v_start    ] = vars[v_start];
    fg[1 + cte_start  ] = vars[cte_start];
    fg[1 + epsi_start ] = vars[epsi_start];

    AD<double> desired_steering_angle;

    for (size_t t = 1; t < N; t++)
        {
            AD<double> x1    = vars[x_start    + t];               // x_start = 0
            AD<double> y1    = vars[y_start    + t];               // y_start = 1*N
            AD<double> psi1  = vars[psi_start  + t];               // psi_start = 2*N
            AD<double> v1    = vars[v_start    + t];               // v_start = 3*N
            AD<double> cte1  = vars[cte_start  + t];               // cte_start = 4*N
            AD<double> epsi1 = vars[epsi_start + t];               // epsi_start = 5*N

            AD<double> x0    = vars[x_start    + t - 1];           // x_start = 0
            AD<double> y0    = vars[y_start    + t - 1];           // y_start = 0
            AD<double> psi0  = vars[psi_start  + t - 1];           // psi_start = 2*N
            AD<double> v0    = vars[v_start    + t - 1];           // v_start = 3*N
            AD<double> cte0  = vars[cte_start  + t - 1];           // cte_start = 4*N
            AD<double> epsi0 = vars[epsi_start + t - 1];           // epsi_start = 5*N

            AD<double> delta = vars[delta_start + t - 1];         // delta (steering)
            AD<double> acc   = vars[acc_start   + t - 1];         // acceleration (throttle/brake)

            // The idea here is to constraint this value to be 0.

            //AD<double> f0 = polyeval(coeffs, x0);
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);

            desired_steering_angle = psi_des0;

            // Setup the rest of the model constraints
            fg[1 + x_start    + t] = x1    - (x0    + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start    + t] = y1    - (y0    + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start  + t] = psi1  - (psi0  - (v0/Lf) * delta * dt);
            fg[1 + v_start    + t] = v1    - (v0    + acc * dt);
            fg[1 + cte_start  + t] = cte1  - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - (v0/Lf) * delta * dt);

         }

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Cost function
    // Define the cost related the reference state and
    // any anything you think may be beneficial.

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++)
    {
      fg[0] += W_cte       * CppAD::pow(vars[cte_start  + t], 2);
      fg[0] += W_epsi      * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += W_v_err     * CppAD::pow(vars[v_start    + t] - ref_v, 2);
      // Added this new part to restrict speed during sharp turns
      fg[0] += W_psi_des_v * CppAD::pow(desired_steering_angle * vars[v_start + t], 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < (N - 1); t++)
    {
      fg[0] += W_delta     * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += W_acc       * CppAD::pow(vars[acc_start   + t], 2);
      // Added new
      fg[0] += W_delta_v   * CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);

    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < (N - 2); t++)
    {
      fg[0] += W_d_delta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += W_d_acc   * CppAD::pow(vars[acc_start + t + 1]   - vars[acc_start + t]  , 2);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  // size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  #define N_STATES      6
  #define N_OUTPUT      2

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 time-steps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N_STATES*N + N_OUTPUT*(N-1);
  // Set the number of constraints
  size_t n_constraints = N_STATES * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start]     = state[0];           // x
  vars[y_start]     = state[1];           // y
  vars[psi_start]   = state[2];           // psi
  vars[v_start]     = state[3];           // v
  vars[cte_start]   = state[4];           // cte
  vars[epsi_start]  = state[5];           // epsi

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lower-limits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++)
 {
    vars_lowerbound[i] = -1.0e10;
    vars_upperbound[i] =  1.0e10;
  }

  // Set Limits for Delta => [25, -25] Degrees
  for (size_t i = delta_start; i < acc_start; i++)
  {
    vars_lowerbound[i] = -0.436332;      // It -25 Degrees but in Radians
    vars_upperbound[i] =  0.436332;      // It +25 Degrees but in Radians
  }

  // Set Limits for acceleration => [1, -1] Degrees
  for (size_t i = acc_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;      // Brake
    vars_upperbound[i] =  1.0;      // Throttle
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]     = state[0];        // x
  constraints_lowerbound[y_start]     = state[1];        // y
  constraints_lowerbound[psi_start]   = state[2];        // v
  constraints_lowerbound[v_start]     = state[3];        // psi
  constraints_lowerbound[cte_start]   = state[4];        // cte
  constraints_lowerbound[epsi_start]  = state[5];        // epsi

  constraints_upperbound[x_start]     = state[0];        // x
  constraints_upperbound[y_start]     = state[1];        // y
  constraints_upperbound[psi_start]   = state[2];        // v
  constraints_upperbound[v_start]     = state[3];        // psi
  constraints_upperbound[cte_start]   = state[4];        // cte
  constraints_upperbound[epsi_start]  = state[5];        // epsi

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  #define LATENCY_COMPENSATION  3

  // Adding LATENCY_COMPENSATION to compensate for the delay
  double Delta_action = solution.x[delta_start+ LATENCY_COMPENSATION];
  // Adding LATENCY_COMPENSATION to compensate for the delay
  double ACC_action   = solution.x[acc_start  + LATENCY_COMPENSATION];

  this->x_predict = {};
  this->y_predict = {};

  for (size_t i = 0; i < N; i++)
  {
     this->x_predict.push_back(solution.x[x_start + i]);
     this->y_predict.push_back(solution.x[y_start + i]);
  }

  return {Delta_action, ACC_action, cost};
}
