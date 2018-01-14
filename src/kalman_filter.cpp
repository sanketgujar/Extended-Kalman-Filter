#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


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
	
	/** The motion noise is taken as 0.
		Predict the new state 
		Please refer equation 11 and 12 in Reference pdf.
		x = F x + u 
		P = F P F' + Q  */

	x_ = F_ * x_;
	MatrixXd Ft_ = F_.transpose();
	P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/** Update the state using real world measurement 
		Please refer equation 13 - 17 in reference pdf.
		y = z − H x'
		S = H P H' + R 
		K = P H' S-1
		x = x + K y 
		P = (I − K H ) P */

	VectorXd y    = z  - H_ * x_;
	MatrixXd Ht_  = H_.transpose();
	MatrixXd S_   = H_ * P_ * Ht_ + R_ ;
	MatrixXd Sin_ = S_.inverse(); 
	MatrixXd K_   = P_ * Ht_ * Sin_; //common gain
	x_ = x_ + K_ * y;
	
	//covariance
	MatrixXd I_ = MatrixXd::Identity(x_.size() ,x_.size());
	P_ = (I_ - K_ * H_) * P_;

}	

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/** Update the state using real world measurement 
		Please refer equation 13 - 17 in reference pdf.
		z' = h(x') + ω
		y = z − z' 
		S = H P H' + R 
		K = P H' S-1
		x = x + K y 
		P = (I − K H ) P */

	//convert the cartesian to polar coordinates	
	double rho = sqrt( pow(x_(0),2) + pow(x_(1),2));
	double phi = atan2( x_(1) , x_(0) );
	double rho_dot = 0 ;

	if ( fabs(rho)  > 0.0001 )
		rho_dot = ( x_(0) * x_(2) + x_(1) * x_(3) ) / rho ;
	
	VectorXd pred_z(3); 
	pred_z << rho , phi , rho_dot ;
	VectorXd y = z - pred_z; 
	
	while ( y(1) > M_PI || y(1) < - M_PI){
		if( y(1) > M_PI)
			y(1) -= M_PI;
		else
			y(1) += M_PI;	 
	}

	MatrixXd Ht_  = H_.transpose();
	MatrixXd S_   = H_ * P_ * Ht_ + R_ ;
	MatrixXd Sin_ = S_.inverse(); 
	MatrixXd K_   = P_ * Ht_ * Sin_; //common gain
	x_ += K_ * y;
	
	//covariance
	MatrixXd I_ = MatrixXd::Identity(x_.size(),x_.size());
	P_ = (I_ - K_ * H_) * P_;
}	
