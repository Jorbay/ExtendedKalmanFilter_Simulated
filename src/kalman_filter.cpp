#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
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
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  
  MatrixXd y_ = z - (H_*x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Sin = S.inverse();
  MatrixXd K = P_*Ht*Sin;

  x_ = x_ + K*y_;
  P_ = (MatrixXd::Identity(4,4)-K*H_)*P_;
  
  //The below is for testing 
  //MatrixXd tester = MatrixXd(4,1);
  //tester << z(0),z(1), x_(2),x_(3);
  //x_ = tester; 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);


  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  float rhoDot = (px*vx + py*vy)/rho;

  VectorXd obs = VectorXd(3);
  obs << rho,phi,rhoDot;

  if (fabs(py) < 0.1){
	std::cout << "UpdateEKF() - Error - Division by Zero" << std::endl;
	std::cout << rho << " " << phi << " " << rhoDot<< std::endl;
  }

  else{

  MatrixXd y_ = z - obs;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Sin = S.inverse();
  MatrixXd K = P_*Ht*Sin;

  x_ = x_ + K*y_;
  P_ = (MatrixXd::Identity(4,4)-K*H_)*P_;
  }

  //The below is for testing 
  //MatrixXd tester = MatrixXd(4,1);
  //tester << cos(z(1))*z(0),sin(z(1))*z(0), x_(2),x_(3);
  //x_ = tester; 


  

}
