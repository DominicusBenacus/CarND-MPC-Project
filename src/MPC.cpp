#include "MPC.h"
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <limits>
#include <cstddef>
#include <iostream>


using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 9;
double dt = 0.10;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Inizialize the model variables
// the factor 0.447 convertsthe 60 mph into m/s
double ref_v = 60*0.447; 

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable
    // values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (uint64_t i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 10 * CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }
    for (uint64_t i = 0; i < N - 1; i++) {
      fg[0] += 10 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }
    for (uint64_t i = 0; i < N - 2; i++) {
      fg[0] += 600 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    // fg[1] is the storage for variables
    // this declaration influence all other access steps
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

   //// The rest of constraints
   //for (uint64_t i = 0; i < N - 1; i++) {
   //  // State at time (i+1)
   //  AD<double> x1 = vars[x_start + i + 1];
   //  AD<double> y1 = vars[y_start + i + 1];
   //  AD<double> psi1 = vars[psi_start + i + 1];
   //  AD<double> v1 = vars[v_start + i + 1];
   //  AD<double> cte1 = vars[cte_start + i + 1];
   //  AD<double> epsi1 = vars[epsi_start + i + 1];

   //  // State at time (i)
   //  AD<double> x0 = vars[x_start + i];
   //  AD<double> y0 = vars[y_start + i];
   //  AD<double> psi0 = vars[psi_start + i];
   //  AD<double> v0 = vars[v_start + i];
   //  AD<double> cte0 = vars[cte_start + i];
   //  AD<double> epsi0 = vars[epsi_start + i];

   //  // Actuation at time i
   //  AD<double> delta = vars[delta_start + i];   
   //  AD<double> a = vars[a_start + i];
   //  // Actuation at time i>0
   //  // go three time steps back to take the latency into account
   //  if (i > 1) { 
   //    a = vars[a_start + i - 1];
   //    delta = vars[delta_start + i -1];
   //  }

      for (uint64_t i = 1; i < N; i++) {
        AD<double> x1 = vars[x_start + i];
        AD<double> x0 = vars[x_start + i - 1];
        AD<double> y1 = vars[y_start + i];
        AD<double> y0 = vars[y_start + i - 1];
        AD<double> psi1 = vars[psi_start + i];
        AD<double> psi0 = vars[psi_start + i - 1];
        AD<double> v1 = vars[v_start + i];
        AD<double> v0 = vars[v_start + i - 1];
        AD<double> cte1 = vars[cte_start + i];
        AD<double> cte0 = vars[cte_start + i - 1];
        AD<double> epsi1 = vars[epsi_start + i];
        AD<double> epsi0 = vars[epsi_start + i - 1];
        AD<double> a = vars[a_start + i - 1];
        AD<double> delta = vars[delta_start + i - 1];
        if (i > 1) {   // use previous actuations (to account for latency)
          a = vars[a_start + i - 2];
          delta = vars[delta_start + i - 2];}
      
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0/Lf* delta * dt);
      fg[2 + v_start + i] = v1 - (v0 + a * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =  epsi1 - ((psi0 - psides0) + v0/Lf* delta * dt);
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
  
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
 const  double x      = state[0];
 const  double y      = state[1];
 const  double psi    = state[2];
 const  double v      = state[3];
 const  double cte    = state[4];
 const  double epsi   = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
 const size_t n_vars = 6 * N + 2 * (N - 1);
  // TODO: Set the number of constraints
 const size_t n_constraints = 6*N;
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint64_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all kinematic model states (6) upper and lowerlimits
  // max negative and positive value of type c++ double
  for (uint64_t i = 0; i < delta_start; i++) {
    //vars_lowerbound[i] = -1.0e19;
    //vars_upperbound[i] = 1.0e19;    
    vars_lowerbound[i] = -std::numeric_limits<double>::max();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  // Set upper and lower limits steering angle delta
  // are set to -25 and 25 degrees --> values are coded in radiants
  // This is a common vehicle contrain
  for (uint64_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Set upper and lower limitsof vehcile control acceleration
  for (uint64_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint64_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // UPPER BOUND
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  // LOWER BOUND
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
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
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  for (uint64_t i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}
