#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  float dt;
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
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noise

  */

  H_laser_ << 1,0,0,0,
             0,1,0,0;


  //Now the transition matrix F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,1,0,
  			 0,1,0,1,
  			 0,0,1,0,
  			 0,0,0,1;

  //Now the covriance matrix P
  ekf_.P_ =MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
  			0,1,0,0,
  			0,0,1000,0,
  			0,0,0,1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      || * Initialize the state ekf_.x_ with the first measurement.
      || * Create the covariance matrix.
      || * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // The x = rho.cos(theta) y = rho.sin(theta)
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      ekf_.x_ << rho*cos(phi), rho*sin(phi),0 , 0 ;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],0,0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     || * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
     ||* Update the process noise covariance matrix.
     || * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = ( measurement_pack.timestamp_ - previous_timestamp_ ) / 1000000.0 ;  
  //To convert timestamp to from milliseconds to seconds 

  //The state transition matrix should be time dependant.
  ekf_.F_(0,2) = dt;
  ekf_.F_(0,4) = dt;

  //Process Covariance matrix...
  // As we don't know the acceleration we add it to the noise part
  // Q is used for random acceleration vector.
  // Q is the expectation value of the noise vector and its transpose.
  // Q = G * Qv * G' 
  // The matrix of Q is given in eqn(40) in reference pdf.

  float dt_2 = pow(dt,2);
  float dt_3 = pow(dt,3);
  float dt_4 = pow(dt,4);

  float noise_ax  = 9 , noise_ay = 9;
  ekf_.Q_ << MatrixXd(2,2);
  ekf_.Q_ << (dt_4/4)*noise_ax, 0 , (dt_3/2)*noise_ax , 0,
  			 0, (dt_4 /4)*noise_ay , 0 , (dt_3/2)*noise_ay,
  			 (dt_3/2)*noise_ax, 0 ,dt_2*noise_ax , 0 ,
  			 0 , (dt_3/2)*noise_ay , 0 , dt_2*noise_ay;



  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
