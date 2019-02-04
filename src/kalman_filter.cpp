#include "kalman_filter.h"
#define pi 3.14159265358979323846

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd z_pred = ekf_.H_laser_ * ekf_.x_;
   VectorXd y = z - z_pred;
   MatrixXd Ht = ekf_.H_laser_.transpose();
   MatrixXd S = ekf_.H_laser_ * ekf_.P_ * Ht + ekf_.R_laser_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = ekf_.P_ * Ht;
   MatrixXd K = PHt * Si;

   //new estimate
   ekf_.x_ = ekf_.x_ + (K * y);
   long x_size = ekf_.x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   ekf_.P_ = (I - K * ekf_.H_) * ekf_.P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   VectorXd z_pred;
   z_pred<<(sqrt(ekf_.x_[0]*sqrt(ekf_.x_[0]+sqrt(ekf_.x_[1]*sqrt(ekf_.x_[1])),
              (atan2(ekf_.x_[1]/ekf_.x_[0])),
              ((ekf_.x_[0]*sqrt(ekf_.x_[2]+sqrt(ekf_.x_[1]*sqrt(ekf_.x_[3])/sqrt(ekf_.x_[0]*sqrt(ekf_.x_[0]+sqrt(ekf_.x_[1]*sqrt(ekf_.x_[1]));

   if (z_pred[1]>pi){
     z_pred[1] -= 2*pi;
   }
   else: if(z_pred[1]<-pi){
     z_pred[1] += 2*pi;
   }
   VectorXd y = z - z_pred;
   MatrixXd Ht = ekf_.H_j_.transpose();
   MatrixXd S = ekf_.H_j_ * ekf_.P_ * Ht + ekf_.R_radar_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = ekf_.P_ * Ht;
   MatrixXd K = PHt * Si;

   //new estimate
   VectorXd y_cartesian;
   y_cartesian << y[0]*cos(y[1]),
                  y[0]*sin(y[1]),
                  y[2]*cos(y[1]),
                  y[2]*sin(y[1]);
   ekf_.x_ = ekf_.x_ + (K * y_cartesian);
   long x_size = ekf_.x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);
   ekf_.P_ = (I - K * ekf_.H_j_) * ekf_.P_;
}
