# SELF DRIVING CAR NANODEGREE
# project5. Extended Kalman Filter


[//]: # (Image References)

[image1-1]: ./images/laser_radar.png "raser and radar characterastics"

[image2-1]: ./images/kalman_filter_1.jpg "About Kalman Filter"
[image2-2]: ./images/kalman_filter_2.jpg "About Kalman Filter"
[image2-3]: ./images/kalman_filter_3.jpg "About Kalman Filter"
[image2-4]: ./images/kalman_filter_4.jpg "About Kalman Filter"
[image2-5]: ./images/kalman_filter_5.jpg "About Kalman Filter"

[image3-1]: ./images/extended_kalman_filter_1.jpg "About Extended Kalman Filter"
[image3-2]: ./images/extended_kalman_filter_2.jpg "About Extended Kalman Filter"
[image3-3]: ./images/extended_kalman_filter_3.jpg "About Extended Kalman Filter"
[image3-4]: ./images/extended_kalman_filter_4.jpg "About Extended Kalman Filter"
[image3-5]: ./images/extended_kalman_filter_5.jpg "About Extended Kalman Filter"
[image3-6]: ./images/extended_kalman_filter_6.jpg "About Extended Kalman Filter"
[image3-7]: ./images/extended_kalman_filter_7.jpg "About Extended Kalman Filter"
[image3-8]: ./images/extended_kalman_filter_8.jpg "About Extended Kalman Filter"
[image3-9]: ./images/extended_kalman_filter_9.jpg "About Extended Kalman Filter"

[image4-1]: ./images/code_flow.png "Code Flow"

[image5-1]: ./images/result.png "Result"



## 1. Abstraction

The object of this project is to detect bicycle around me (supposing I am driving) by implementing Extended Kalman Filter with C++ 

Red circle is sensor data from Lidar,

Blue circle is sensor data from Radar,

Green triangle is result of calculation that predict its position & direction

Object of this project is predict bicycle's position by using Extended Kalman Filter

and its RMSE should be under [.11, .11, 0.52, 0.52]

*RMSE : Root Mean Squared Error*

- Udacity provided simulator and sensor measurement data

- It generated noise Lidar, Radar sersor measurements of the position and velocity of object

- For predict its position I had to fusion two sensors

## 2. Related Study

#### 1) Sensors

① Radar, Lidar strengths and weaknesses

#### 2) Kalman Filter

② Iteration of predict and measurement update

#### 3) Extended Kalman Filter

① Sensor Fusion (predict and update using both radar and raser sensors)


## 3. Details

#### 1) Content Of This Repo

- ```src``` a directory with the project code
    - ```main.cpp``` : reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
    - ```FusionEKF.cpp``` : initializes the filter, calls the predict function, calls the update function
    - ```kalman_filter.cpp``` : defines the predict function, the update function for lidar, and the update function for radar
    - ```tools.cpp``` : a function to calculate RMSE and the Jacobian matrix
- ```data``` a directory with two input files, provided by Udacity


#### 2) Code Flow

<img src="./images/code_flow.png" width="600">

① main.cpp




② FusionEKF.h

```c++
class FusionEKF {
 public:

  FusionEKF();

  virtual ~FusionEKF();
  
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  KalmanFilter ekf_;

 private:

  bool is_initialized_;

  long long previous_timestamp_;

  Tools tools;
  
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  double noise_ax;
  double noise_ay;

};
```

③ kalman-filter.h

```c++
class KalmanFilter {

 Tools tools;
  
 public:

  KalmanFilter();

  virtual ~KalmanFilter();

  // Initializes Kalman filter (x, P, F, H, Hj, R, R_ekf, Q, I)
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_in, 
            Eigen::MatrixXd &R_ekf_in, Eigen::MatrixXd &Q_in);

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

④ measurement_package.h

```c++
class MeasurementPackage {
 public:

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  // It include px,py for LASER, rho, theta, rho_dot for RADAR
  Eigen::VectorXd raw_measurements_;
  
};
```

⑤ tools.h

```c++
class Tools {
 public:

  Tools();

  virtual ~Tools();

  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};
```

## 4. Results

I have done one simulation and got this result

px,py means position of x,y

vx, vy means calculated velocity of x,y

![alt text][image5-1]
<img src="./images/result.png" width="300">

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.1164 |
|  py   | 0.2811 |
|  vx   | 0.4542 |
|  vy   | 0.7827 |


## 5. Discussion

#### 1) Test only using one sensor

I will test using only one sensor (Lidar or Radar)

Wonder how that result is diffrent from using both sensors



