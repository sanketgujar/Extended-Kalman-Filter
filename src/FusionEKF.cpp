#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // Measurment Matrix for laser      
  H_laser_ << 1,0,0,0,
             0,1,0,0;


  //Jacobian measurment for Radar
  Hj_ << 1,1,0,0,
  		 1,1,0,0,
  		 1,1,1,1;           

  //The transition matrix F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,1,0,
  			 0,1,0,1,
  			 0,0,1,0,
  			 0,0,0,1;

  //The covariance matrix P
  ekf_.P_ =MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
  			0,1,0,0,
  			0,0,1000,0,
  			0,0,0,1000;
  

}

FusionEKF::~FusionEKF() {}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
	//Initializing the state ekf_.x_ with the first measurement.
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //Converting Polar coordinates to cartesian
      // The x = rho.cos(theta) y = rho.sin(theta)
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      double ro_dot = measurement_pack.raw_measurements_(2);
      double x = rho*cos(phi);
      ekf_.x_ << rho*cos(phi) , rho*sin(phi) , ro_dot * cos( phi ) , ro_dot * sin( phi );
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //Laser Update
      ekf_.x_ << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),0,0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //To convert timestamp to from microsecond to second 
  float dt = ( measurement_pack.timestamp_ - previous_timestamp_ ) / 1000000.0 ;  
  previous_timestamp_ = measurement_pack.timestamp_;
  	
  /**The state transition matrix should be time dependent.
  	 x' = F x + ν	
  	 Refer Equation 21 in reference pdf.	
  */
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  /**Process Covariance matrix...
  	 Q is used for random acceleration vector, 
  	 as we don't know the acceleration we add it to the noise part
  	 Q is the expectation value of the noise vector and its transpose.
  	 Q = G * Qv * G' 
  	 The matrix of Q is given in eqn(40) in reference pdf.
  */			 

  float dt_2 = pow(dt,2);
  float dt_3 = pow(dt,3);
  float dt_4 = pow(dt,4);
  float noise_ax  = 9 , noise_ay = 9;


  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;



  //Predicting the new position            
  ekf_.Predict();



  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
  	We have to choose the measurement Function (H) and noise (R) for the update
  	for two types of sensors and then call the update function 
  */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

  	// Radar update..
  	Tools tools;
  	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  
  } else {
  
    // Laser updates
    ekf_.H_ = H_laser_; 
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
