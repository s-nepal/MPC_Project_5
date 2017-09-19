#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

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

double medianVelocity = 70;
size_t xStart = 0;
size_t yStart = xStart + N;
size_t psiStart = yStart + N;
size_t vStart = psiStart + N;
size_t cteStart = vStart + N;
size_t psiErrorStart = cteStart + N;
size_t deltaStart = psiErrorStart + N;
size_t accelStart = deltaStart + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` is a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    for(unsigned int i = 0; i < N; i++) {
      fg[0] += 3000*CppAD::pow(vars[cteStart + i], 2);
      fg[0] += 3000*CppAD::pow(vars[psiErrorStart + i], 2);
      fg[0] += CppAD::pow(vars[vStart + i] - medianVelocity, 2);
    }

    for(unsigned int i = 0; i < N - 1; i++) {
      fg[0] += 5*CppAD::pow(vars[deltaStart + i], 2);
      fg[0] += 5*CppAD::pow(vars[accelStart + i], 2);
      fg[0] += 700*CppAD::pow(vars[deltaStart + i] * vars[vStart+i], 2);
    }

    for(unsigned int i = 0; i < N - 2; i++) {
      fg[0] += 200*CppAD::pow(vars[deltaStart + i + 1] - vars[deltaStart + i], 2);
      fg[0] += 10*CppAD::pow(vars[accelStart + i + 1] - vars[accelStart + i], 2);
    }

    fg[1 + xStart] = vars[xStart];
    fg[1 + yStart] = vars[yStart];
    fg[1 + psiStart] = vars[psiStart];
    fg[1 + vStart] = vars[vStart];
    fg[1 + cteStart] = vars[cteStart];
    fg[1 + psiErrorStart] = vars[psiErrorStart];

    // other constraints
    for (unsigned i = 1; i < N; i++) {
      AD<double> x1 = vars[xStart + i];
      AD<double> x0 = vars[xStart + i - 1];
      AD<double> y1 = vars[yStart + i];
      AD<double> y0 = vars[yStart + i - 1];
      AD<double> psi1 = vars[psiStart + i];
      AD<double> psi0 = vars[psiStart + i - 1];
      AD<double> v1 = vars[vStart + i];
      AD<double> v0 = vars[vStart + i - 1];
      AD<double> cte1 = vars[cteStart + i];
      AD<double> cte0 = vars[cteStart + i - 1];
      AD<double> psiError1 = vars[psiErrorStart + i];
      AD<double> psiError0 = vars[psiErrorStart + i - 1];
      AD<double> a = vars[accelStart + i - 1];
      AD<double> delta = vars[deltaStart + i - 1];

      // simulate latency by referring to the previous actuation value
      if (i > 1) {   
        a = vars[accelStart + i - 2];
        delta = vars[deltaStart + i - 2];
      }

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + xStart + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + yStart + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psiStart + i] = psi1 - (psi0 - v0/Lf * delta * dt);
      fg[1 + vStart + i] = v1 - (v0 + a * dt);
      fg[1 + cteStart + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(psiError0) * dt));
      fg[1 + psiErrorStart + i] = psiError1 - ((psi0 - psides0) - v0/Lf * delta * dt);
    }

  }
};


// MPC class definition implementation.
// Constructor & Destructor
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double psiError = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for(unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  vars[xStart] = x;
  vars[yStart] = y;
  vars[psiStart] = psi;
  vars[vStart] = v;
  vars[cteStart] = cte;
  vars[psiErrorStart] = psiError;


  // Set all non-actuators upper and lower limits
  // to extreme values
  for(unsigned int i = 0; i < deltaStart; i++) {
    vars_lowerbound[i] = -99999999;
    vars_upperbound[i] = 99999999;
  }

  // Set delta to within [-25deg, 25deg] in radians
  for(unsigned int i = deltaStart; i < accelStart; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Set acceleration to within [-1, 1]
  for(unsigned int i = accelStart; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Set the lower and upper limits for the constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for(unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[xStart] = x;
  constraints_lowerbound[yStart] = y;
  constraints_lowerbound[psiStart] = psi;
  constraints_lowerbound[vStart] = v;
  constraints_lowerbound[cteStart] = cte;
  constraints_lowerbound[psiErrorStart] = psiError;

  constraints_upperbound[xStart] = x;
  constraints_upperbound[yStart] = y;
  constraints_upperbound[psiStart] = psi;
  constraints_upperbound[vStart] = v;
  constraints_upperbound[cteStart] = cte;
  constraints_upperbound[psiErrorStart] = psiError;

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

  result.push_back(solution.x[deltaStart]);
  result.push_back(solution.x[accelStart]);

  for(unsigned int i = 0; i < N-1; i++) {
    result.push_back(solution.x[xStart + i + 1]);
    result.push_back(solution.x[yStart + i + 1]);
  }

  return result;

  // return {};
}
