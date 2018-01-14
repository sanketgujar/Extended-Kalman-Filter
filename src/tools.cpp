#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	//Please see equation 96 in reference pdf for Jacobian matrix..
	MatrixXd Hj_(3,4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);


	float a1 = ( pow( px , 2 ) + pow( py , 2 ) );
	if ( fabs(a1) < 0.0001){
		cout << "Division by Zero Error in Tools::CalculateJacobian ";
		return Hj_;
	}
	float a2 = sqrt( a1 );
	float a3 = a1 * a2 ;

	Hj_ << px / a2 , py / a2 , 0 , 0 ,
		   -py / a1 , px / a1 , 0 , 0 ,
		   (py * (vx * py - vy * px )) / a3 , (px * (vy * px - vx * py ))/a3 , px / a2 , py / a2 ;
	
	return Hj_; 

}
