#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
    // Calculate the RMSE
  
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size() == 0){
      cout << "ERROR - CalculateRMSE () - The estimations vector is empty" << endl;
      return rmse;
    }

    if(ground_truth.size() == 0){
      cout << "ERROR - CalculateRMSE () - The ground-truth vector is empty" << endl;
      return rmse;
    }

    unsigned int n = estimations.size();
    if(n != ground_truth.size()){
      cout << "ERROR - CalculateRMSE () - The ground-truth and estimations vectors must have the same size." << endl;
      return rmse;
    }

    for(unsigned int i=0; i < estimations.size(); ++i){
      VectorXd diff = estimations[i] - ground_truth[i];
      diff = diff.array()*diff.array();
      rmse += diff;
    }

    rmse = rmse / n;
    rmse = rmse.array().sqrt();
    return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  // Calculate a Jacobian here.
  
  MatrixXd Hj(3,4);
  
  if ( x_state.size() != 4 ) {
    cout << "ERROR - CalculateJacobian () - The state vector must have size 4." << endl;
    return Hj;
  }
  
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);
  
  double c1 = sqrt(px*px + py*py);
  double c2 = c1*c1;
  double c3 = c2*c1;
  
  if(fabs(c1) < 0.0001){
  cout << "ERROR - CalculateJacobian () - Division by Zero" << endl;
  return Hj;
  }
  
  Hj << px/c1, py/c1, 0 ,0,
        -py/c2, -px/c2, 0, 0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c1, py/c1;
  
  return Hj;
  
}
