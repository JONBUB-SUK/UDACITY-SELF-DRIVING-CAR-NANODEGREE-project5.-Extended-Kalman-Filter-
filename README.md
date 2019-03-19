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
main.cpp
fusionEKF.h
kalman-filter.h
measurement_package.h
tools.h

# Results
...

# Conclusion & Discussion
...
