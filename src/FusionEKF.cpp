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
  // It is H for radar
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //initializing the FusionEKF.
  //Set the process and measurement noises
  
  noise_ax = 9.;
  noise_ay = 9.;

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
   
     // initialize the state ekf_.x_ with the first measurement.
     // Create the covariance matrix.

    // first measurement
    cout << "Initializing EKF" << endl;
    VectorXd x(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
    float px;
    float py;
//    At first measurement, we assume velocity is 0.
//    float vx;
//    float vy;
    
    float rho;
    float phi;
//    float rho_dot;
      
	rho = measurement_pack.raw_measurements_[0];
	phi = measurement_pack.raw_measurements_[1];      
//    rho_dot = measurement_pack.raw_measurements_[2];      
      
    px = rho * cos(phi);
    py = rho * sin(phi);
//    vx = rho_dot * cos(phi);
//    vy = rho_dot * sin(phi);
      
    x << px, py, 0.f, 0.f;
    
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
	
	float px;
    float py;
//    float vx;
//    float vy;    
      
	px = measurement_pack.raw_measurements_[0];      
	py = measurement_pack.raw_measurements_[1];
//	vx = measurement_pack.raw_measurements_[2];     
//	vy = measurement_pack.raw_measurements_[3];     

    x << px, py, 0.f, 0.f;
      
    }
    cout << "End of x information Initializing" << endl;
    
	previous_timestamp_ = measurement_pack.timestamp_;

    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;
    
    // At the prediction step, I will add dt to F matrix
    // Now I dropped dt because dt changes whenever measures
    MatrixXd F(4, 4);
    F << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
 
    H_laser_ << 1, 0, 0, 0,
               0, 1, 0, 0;
    
    MatrixXd Q(4,4);

    // I added Hj, to init function
    // At the Update step, I will choose H, R depending on radar or laser
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
    cout << "End of Initializing EKF" << endl;
    return;
  }

  /**
   * Prediction
   */

   //Update the state transition matrix F according to the new elapsed time.
   //Time is measured in seconds.
   //Update the process noise covariance matrix.
   //Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 /2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
  
  
  ekf_.Predict();

  /**
   * Update
   */

  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
       
  } else {
    // Laser updates

    ekf_.Update(measurement_pack.raw_measurements_);    
    
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
