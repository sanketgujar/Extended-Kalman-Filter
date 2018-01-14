#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
	//The motion noise is taken as 0.
	x_ = F_ * x_;
	MatrixXd Ft_ = F_.transpose();
	P_ = F_ * P_ * Ft_ + Q_;			
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */	
	VectorXd y    = z  - H_ * x_;
	MatrixXd Ht_  = H_.transpose();
	MatrixXd S_   = H_ * P_ * Ht_ + R_ ;
	MatrixXd Sin_ = S_.inverse(); 
	MatrixXd K_   = P_ * Ht_ * Sin_; //common gain

	x_ += K_ * y;
	
	//covariance
	MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I_ - K_ * H_) * P_;
}	

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	// z = h(x') + ω
	//convert the cartesian to polar coordinates
	float rho = sqrt( pow(x_(0),2) + pow(x_(1),2));
	float phi = atan2( x_(1) , x_(2) );
	float rho_dot = 0 ;
	if ( fabs(rho)  < 0.0001 )
		rho_dot = ( x_(0) * x_(2) + x_(1) * x_(3) ) / rho ;

	VectorXd pred_z(3); 
	pred_z << rho , phi , rho_dot ;
	VectorXd y = z - pred_z; 
	MatrixXd Ht_  = H_.transpose();
	MatrixXd S_   = H_ * P_ * Ht_ + R_ ;
	MatrixXd Sin_ = S_.inverse(); 
	MatrixXd K_   = P_ * Ht_ * Sin_; //common gain

	x_ += K_ * y;
	
	//covariance
	MatrixXd I_ = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I_ - K_ * H_) * P_;


}	
