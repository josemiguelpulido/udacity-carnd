#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0) {
    cout << "CalculateRMSE - estimation vector size zero" << endl;
    return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE - estimation vector and ground truth vector have different sizes" << endl;
    return rmse;
  }

  //accumulate squared residuals
  VectorXd residuals(4);
  residuals << 0,0,0,0;
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd diff(4);
    diff << estimations[i] - ground_truth[i];
    VectorXd residual(4);
    residual = diff.array()*diff.array();
    residuals += residual;
    
  }

  //calculate the mean
  // ... your code here
  residuals /= estimations.size();

  //calculate the squared root
  // ... your code here
  rmse = residuals.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE 

  //check division by zero
  if (px == 0 && py == 0) {
    cout << "division by zero" << endl;
    return Hj;
  }
  
  //compute the Jacobian matrix
  float ro_square = px*px + py*py;
  float ro = sqrt(ro_square);
  float ro_cube = ro*ro*ro;
  Hj <<
    px/ro, py/ro, 0, 0,
    -py/ro_square, px/ro_square, 0, 0,
    py*(vx*py - vy*px)/ro_cube, py*(vy*px - vx*py)/ro_cube, px/ro, py/ro;
      

  return Hj;

}
