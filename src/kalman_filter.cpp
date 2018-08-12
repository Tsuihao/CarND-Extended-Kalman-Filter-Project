#include "kalman_filter.h"
#include <cmath> // sqrt, atan2

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
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Eigen::MatrixXd I = MatrixXd::Identity(4, 4);
  y = z - H_ * x_;
  S = R_ + H_ * P_ * H_.transpose();
  K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_ * (I - K * H_).transpose() + K * R _* K.transpose();

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Eigen::VectorXd observation = z; // use the dimension of z
  Observe(x_, observation);
  y = z - observation;




}

void KalmanFilter::Observe(const Eigen::VectorXd& state_vector,
                           Eigen::VectorXd& result) {
  double px = state_vector(0);
  double py = state_vector(1);
  double vx = state_vector(2);
  double vy = state_vector(3);

  double range = std::sqrt( px*px + py*py );
  double bearing = std::atan2( py/px );
  double range_rate;

  if(!isZero(range))
    range_rate = (px*vx + py*vy)/range;
  else
    range_rate = std::numeric_limites<double>::infinity();

  result<< range,
           bearing,
           range_rate;
}
