#include "kalman_filter.h"
#include <iostream>
#include <math.h>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
  return;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // Go back to polar
  VectorXd z_pred = VectorXd(3);
  z_pred(0) = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  z_pred(1) = atan2(x_[1], x_[0]);
  z_pred(2) = (x_[0]*x_[2] + x_[1]*x_[3])/z_pred(0);

  // Correct slope if necessary
  if (z_pred(1) > PI) {
    z_pred(1) = z_pred(1) - 2*PI;
  }
  if (z_pred(1) < -PI) {
    z_pred(1) = z_pred(1) + 2*PI;
  }

	VectorXd y = z - z_pred;

  // Slope correction again for the estimated error
  if (y(1) > PI) {
    y(1) = y(1) - 2*PI;
  }
  if (y(1) < -PI) {
    y(1) = y(1) + 2*PI;
  }

  // Calculate Kalman gain
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

  return;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Calculate Kalman gain
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  return;

}
