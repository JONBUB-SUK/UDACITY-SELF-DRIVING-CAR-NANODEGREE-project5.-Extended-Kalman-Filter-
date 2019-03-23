# Introduction

The object of this project is to detect bicycle around me (supposing I am driving) by implementing Extended Kalman Filter with C++ 

Red circle is sensor data from Lidar,

Blue circle is sensor data from Radar,

Green triangle is result of calculation that predict its position & direction

Object of this project is predict bicycle's position by using Extended Kalman Filter

and its RMSE should be under [.11, .11, 0.52, 0.52]

* RMSE : Root Mean Squared Error *

- Udacity provided simulator and sensor measurement data

- It generated noise Lidar, Radar sersor measurements of the position and velocity of object

- For predict its position I had to fusion two sensors

# Background Learning
For this project, I had to learn principle of Kalman-Filter

### 1. Sensors

- Radar, Lidar strengths and weaknesses

### 2. Kalman Filter

- Iteration of predict and measurement update

### 3. Extended Kalman Filter

- Sensor Fusion (predict and update using both radar and raser sensors)

(NEED PICTURE)

# Content Of This Repo
- ```src``` a directory with the project code
	- ```main.cpp``` : reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
    - ```FusionEKF.cpp``` : initializes the filter, calls the predict function, calls the update function
    - ```kalman_filter.cpp``` : defines the predict function, the update function for lidar, and the update function for radar
    - ```tools.cpp``` : a function to calculate RMSE and the Jacobian matrix
- ```data``` a directory with two input files, provided by Udacity


# Code Flow

(NEED PICTURE)

# Summary Of Each File
1. FusionEKF.h

```c++
class FusionEKF {
 public:
  
  // Constructor
  FusionEKF();

  // Destructor. 
  virtual ~FusionEKF();
  
  // It is main function that perform predict next position and calculate position by inputing sensor data
  // Firstly, if it has first data input, we have to initialize data by using ekf_.init() function
  // Secondly, predict next x,P by using only dt, Q noise
  // Finally, calculate current position by using sensor data
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // We need KalmanFilter object for ProcessMeasurement
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
 
 // Need Tools object to calculate RMSE
 Tools tools;
  
 public:

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  // Initializes Kalman filter (x, P, F, H, Hj, R, R_ekf, Q, I)
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &R_ekf_in, Eigen::MatrixXd &Q_in);

  // Predict position,velocity by using only F matrix (dt)
  void Predict();

  // Calculate position, velocity by using Laser measurement data
  void Update(const Eigen::VectorXd &z);

  // Calculate position, velocity by using Radar measurement data
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
 
  // Select sensor type LASER/RADAR
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // For calculating dt
  long long timestamp_;

  // It include px,py for LASER, rho, theta, rho_dot for RADAR
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

I have done one simulation and got this result

px,py means position of x,y

vx, vy means calculated velocity of x,y

(NEED PICTURE)

*Test One*

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.1164 |
|  py   | 0.2811 |
|  vx   | 0.4542 |
|  vy   | 0.7827 |


# Conclusion & Discussion

I will test using only one sensor (Lidar or Radar)

Wonder how that result is diffrent from using both sensors

2. y 값 차이가 커서 손좀 봐야할듯
