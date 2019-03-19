# Introduction
This is sixth project of Udacity Self-Driving Car Nanodegree program (Extended Kalman Filter)
Object is to detect bicycle around me (supposing I am driving)
Red circle is sensor data from Lidar,
Blue circle is sensor data from Radar,
Green triangle is result of calculation that predict its position & direction
Object of this project is predict bicycle's position by using Extended Kalman Filter
and its RMSE should be under ()
Udacity provided starter code and sensor measurement data
....

# Background Learning
For this project, I had to learn principle of Kalman-Filter
.....

# Content Of This Repo
- 'src' a directory with the project code
	- main.cpp : reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
    - FusionEKF.cpp : initializes the filter, calls the predict function, calls the update function
    - kalman_filter.cpp : defines the predict function, the update function for lidar, and the update function for radar
    - tools.cpp : a function to calculate RMSE and the Jacobian matrix
- 'data' a directory with two input files, provided by Udacity

# Approach
...

# Code Flow
...

# Summary Of Each File
1. FusionEKF.h

```c++
class FusionEKF {
 public:
  
  // Constructor
  FusionEKF();

  
  // Destructor. 
  virtual ~FusionEKF();
  
  // 설명
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // 설명
  KalmanFilter ekf_;

 private:
 
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  // noise constant
  double noise_ax;
  double noise_ay;

};
```

2. kalman-filter.h

```c++
class KalmanFilter {
 
 // 설명
 Tools tools;
  
 public:

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  // Initializes Kalman filter (x, P, F, H, Hj, R, R_ekf, Q, I)
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &R_ekf_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  // 설명
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  // 설명
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  // 설명
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;
  
  // measurement matrix for Jacobian
  Eigen::MatrixXd Hj_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
  
  // measurement covariance matrix for RADAR
  Eigen::MatrixXd R_ekf_;
  
  // identity matrix
  Eigen::MatrixXd I_;
  
};
```

3. measurement_package.h

```c++
class MeasurementPackage {
 public:
 
  // 설명
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // 설명
  long long timestamp_;

  // 설명
  Eigen::VectorXd raw_measurements_;
  
};
```

4. tools.h

```c++
class Tools {
 public:

  // Constructor
  Tools();

  // Destructor
  virtual ~Tools();

  // Calculating RMSE
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  // Calculating Jacobian
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};
```

# Results
...

# Conclusion & Discussion
...
