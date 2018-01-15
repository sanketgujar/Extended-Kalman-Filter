# Extended Kalman Filter Project 
I utilized a kalman filter to estimate the state of a moving object of interest (bicycle that travels around the vehicle) with noisy lidar and radar measurements. 

Please checkout reference.pdf for Equations used in the code.

The Flow of my code is as shown below

![Alt text](https://github.com/sanketgujar/Extended-Kalman-Filter/blob/master/Output/f2.png)



The 3 important steps are 

1. Initializing the matrices    
   
   On the first measurment we will update the state and covariance matrix.  

2. Predict the state 
   
   Compute the time elapsed between previous and current measurment.
   
   Use the time difference to compute the state and covariance matrix.
   
   Predict the new state and covariance 
   
3. Update the state 
   
   If the measurement is from Laser, set up the Laser matrices and update the new measurment.
   
   If the measurement is from Radar, convert to linear, set up the Radar matrices and update the new measurement.



## Testing the Kalaman Filter in Simulator. 

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

![Alt text](https://github.com/sanketgujar/Extended-Kalman-Filter/blob/master/Output/1.gif)

![Alt text](https://github.com/sanketgujar/Extended-Kalman-Filter/blob/master/Output/2.gif)


## How the Files Relate to Each Other
Here is a brief overview of what happens when you run the code files:

Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp

FusionEKF.cpp takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ instance holds the matrix and vector values and calls the predict and update equations.

The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h. 'kalman_filter.cpp' contains functions for the prediction and update steps.

## Dependencies

The Simulator involved in the project can be downloaded from [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF


INPUT: values provided by the simulator to the c++ program
["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator
["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---


* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
