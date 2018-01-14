#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// Checking if the estimation vector is valid
	if ( estimations.size() != ground_truth.size() || estimations.size() == 0){
		cout << " Invalid estimations matrix passed to Tools::CalculateRMSE  : "<<endl;
		return rmse;
	}

	for (unsigned int i = 0 ; i < estimations.size() ; ++i ){
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**Calculate a Jacobian for Non-Linear Measurement.
	 Please see equation 96 in reference pdf for Jacobian matrix.*/

	MatrixXd Hj_(3,4);
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);


	double a1 = ( pow( px , 2 ) + pow( py , 2 ) );
	if ( fabs(a1) < 0.0001){
		cout << "Division by Zero Error in Tools::CalculateJacobian "<<endl;
		return Hj_;
	}
	double a2 = sqrt( a1 );
	double a3 = a1 * a2 ;

	Hj_ << px / a2 , py / a2 , 0 , 0 ,
		   -py / a1 , px / a1 , 0 , 0 ,
		   (py * (vx * py - vy * px )) / a3 , (px * (vy * px - vx * py ))/a3 , px / a2 , py / a2 ;
	
	return Hj_; 
}
