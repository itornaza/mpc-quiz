// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"

//
// Helper functions
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());
  
  // Vehicle previous state
  double x_0 = state[0];
  double y_0 = state[1];
  double psi_0 = state[2];
  double v_0 = state[3];
  
  // Actuators to the vehicle
  double delta = actuators[0];
  double alpha = actuators[1];
  
  // Next state
  next_state[0] = x_0 + v_0 * cos(psi_0) * dt;
  next_state[1] = y_0 + v_0 * sin(psi_0) * dt;
  next_state[2] = psi_0 + (v_0 / Lf) * delta * dt;
  next_state[3] = v_0 + alpha * dt;
  
  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  // should be [0.212132, 0.212132, 0.798488, 1.3]
  auto next_state = globalKinematic(state, actuators, 0.3);

  std::cout << next_state << std::endl;
}
