#ifndef MPC_H
#define MPC_H

#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


void initParams(char ** argv);
class MPC {
 public:

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
