#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	double px;
    double py;
    double vx;
    double vy;
    
    double rho;
    double phi;
    double rho_dot;
      
	rho = measurement_pack.raw_measurements_[0];
	phi = measurement_pack.raw_measurements_[1];      
	rho_dot = measurement_pack.raw_measurements_[2];      
      
    px = rho * cos(phi);
    py = rho * sin(phi);
    vx = rho_dot * cos(phi);
    vy = rho_dot * sin(phi);
      
    ekf_.x_ << px, py, vx, vy;
    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	
	double px;
    double py;
    double vx;
    double vy;    
      
	px = measurement_pack.raw_measurements_[0];      
	py = measurement_pack.raw_measurements_[1];
	vx = measurement_pack.raw_measurements_[2];     
	vy = measurement_pack.raw_measurements_[3];     

    ekf_.x_ << px, py, vx, vy;
      
    }
	
    previous_timestamp_ = measurement_pack.timestamp_; 
 
 
    MatrixXd P(4, 4); 
    P << 1, 0, 0, 0, 
         0, 1, 0, 0, 
         0, 0, 1000, 0, 
         0, 0, 0, 1000; 


    MatrixXd F(4, 4); 
    F << 1, 0, 0, 0, 
         0, 1, 0, 0, 
         0, 0, 1, 0, 
         0, 0, 0, 1; 
 
    H_laser_ << 1, 0, 0, 0, 
                0, 1, 0, 0; 
 
    MatrixXd Q(4,4); 
    ekf_.Init( x, /*x_in*/  
               P, /*P_in*/  
               F, /*F_in*/ 
               H_laser_, /*H_in*/  
               Hj_, /*Hj_in*/  
               R_laser_, /*R_in*/  
               R_radar_, /*R_ekf_in*/  
               Q ); /*Q_in*/ 

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0,2) = dt; 
  ekf_.F_(1,3) = dt; 

  
  double noise_ax = 9.0;
  double noise_ay = 9.0;
  
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;
  
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 /2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
  
  
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
       
  } else {
    // TODO: Laser updates

    ekf_.Update(measurement_pack.raw_measurements_);    
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
