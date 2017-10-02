# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

   I built a kalman filter to estimate the state of a moving object with noisy simulated LIDAR and RADAR data. The state was found to have an RMSE less than the noise of either the simulated LIDAR or RADAR sensors. This project was done as part of Udacity's Self-Driving Car Engineer Nanodegree Program, specifically, the second semester. This explanation will be broken into the following sections:

1. Motivation
2. Dependencies for Project
3. Code breakdown
4. Results

   If  you have any questions about the code or setup, please feel free to contact me at jorge@jorgeorbay.com. 

##Motivation
   My initial motivation for building this was to complete Udacity's Self Driving Car Nanodegree, a set of classes on programming self driving cars. Udacity was founded by Sebastian Thrun, the founder of Google's self driving car project, and has been a wonderful resource.

   Learning the mechanics of the Extended Kalman filter itself has been a great reward from this project. A Kalman filter is essentially a tool for tracking an object state by utilizing more than just the last update information. I think this is best utilized with a GPS example. When keeping track of your position in Apple Maps, your phone does not just use the last location registered from GPS satellites, which is why your position is normally shown as constant instead of jumping around within a ~5 m circle (the common noise distribution of GPS). Instead, your phone keeps a running track on your position which it constantly updates (but does not replace) with new GPS information. The mechanics of how Apple Maps updates this information is probably different from this project, but the underlying principles are the same: as the position of an object is updated, each update is associated with a value of expected noise, and the combination of previous state and update results in a new state.

   The "Extended" part of "Extended Kalman filter" refers to the non-linearity of translating RADAR data to positioning in the XY coordinate plane. Because the RADAR data is in polar coordinates, and because the translation from polar coordinates to XY is non-linear, a linear approximation of the RADAR data must be made to maintain the linear translation of Gaussian noise (non-linear translations of Gaussian noise would result in non-Gaussian noise, which would be harder to model). 

##Dependencies
 
   The C++ code and simulator used in this project have necessary dependencies you can find below.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Other Important Dependencies:

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

##Mechanics of the Code

   The code shown is based off of a framework provided by Udacity. My changes are found in ./src/FusionEKF.cpp, kalman_filter.cpp, and tools.cpp. My custom, added code is marked with a TODO flag before every block. The functions in all these files are called in main.cpp, which is run by calling ./build/ExtendedKF. 

   In FusionEKF.cpp, the values for the process covariance, state covariance, process noise mean, and state noise mean are initialized. These first mean values are chosen from the first measurement, which can be seen in lines 77-100.  All these covariance values are just part of the mechanics of modeling the noise of predicting change in the state as it progresses in time. This can be seen in lines 14-64. Also, the measurement matrix and covariance of sensor noise are initialized in this file for each sensor and called in lines 165-185. The calculation for the process covariance, which is dependent on the time differential between updates, is calculated in lines 133-150. 

   In kalman_filter.cpp, the specific calculations for the prediction and update steps of the Kalman filter are implemented. Predict() is the prediction step of every update, and it takes its values from FusionEKF. Update() is the update step for LIDAR measurements. UpdateEKF() is the update step for RADAR measurements, which has a linearized H because of the nonlinear transformation between polar and XY coordinates.

   In tools.cpp, only two functions exist. CalculateRMSE() is used to calculate the root mean square error of the predicted state and ground truth. CalculateJacobian() is used to calculate the Jacobian matrix used to linearize the H matrix of RADAR.

##Results

   If using the direct results of the latest LiDAR measurment without any filtering, I could only achieve a RMSE of the X and Y positions of .201 and .185.

   If using the direct results of the latest RADAR measurement without any filtering, I could only achieve a RMSE of the X and Y positions of .393 and .519.

   If using a Kalman filter on the LiDAR and RADAR data together, I achieved a RMSE of the X and Y positions of .097 and .085, more than twice as accurate than the sensor data itself.

   Thus, the Kalman filter is a success in increasing state precision relative to just using the latest data points directly.
