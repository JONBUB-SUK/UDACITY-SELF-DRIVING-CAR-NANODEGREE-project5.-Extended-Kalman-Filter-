#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_in, MatrixXd &R_ekf_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Hj_ = Hj_in;
  R_ = R_in;
  R_ekf_ = R_ekf_in;
  Q_ = Q_in;
  // I added Identity matrix to kalman_filter class for convinience
  I_ = Eigen::MatrixXd::Identity(4,4);
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  
  MatrixXd Ft = F_.transpose();  
  
  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  
  // If px ==0 or py ==0, skip that information
  if( px == 0. && py == 0. )
    return;
  
  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float rho_dot = (px*vx + py*vy) / rho;

  Hj_ = tools.CalculateJacobian(x_);
  
  VectorXd h_x_prime = VectorXd(3);
  h_x_prime << rho, theta, rho_dot;
  
  VectorXd y = z - h_x_prime;
  
  // After calculate y, theta can be over plus minus phi, so I have to manipulate theta to in range -phi~phi
  if (y[1] > M_PI){
    y[1] -= 2.f*M_PI;
  }
  else if (y[1] < -M_PI){
    y[1] += 2.f*M_PI;
  }
  
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Hjt + R_ekf_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Hjt * Si;
  
  x_ = x_ + (K * y);
  P_ = (I_ - K * Hj_) * P_;  
   
}
