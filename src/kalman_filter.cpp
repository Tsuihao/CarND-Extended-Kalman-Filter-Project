#include "kalman_filter.h"
#include <cmath> // sqrt, atan2
#include "tools.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

  Set_predict_params(x_in, P_in, F_in, Q_in);
  Set_update_params(H_in, R_in);
}

void KalmanFilter::Set_predict_params(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
                                Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in) {

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Set_update_params(Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in) {

  H_ = H_in;
  R_ = R_in;
}

void KalmanFilter::Set_x(const Eigen::VectorXd &x_in){

  x_ = x_in;
}

void KalmanFilter::Set_P(const Eigen::MatrixXd &P_in){

  P_ = P_in;
}

const Eigen::VectorXd& KalmanFilter::Get_x() const{
  return x_;
}

const Eigen::MatrixXd& KalmanFilter::Get_P() const{
  return P_;
}


void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  if(verbose) {std::cout<<"[Kalman filter]: predict x_=\n"<<x_<<std::endl;}
  P_ = F_ * P_ * F_.transpose() + Q_;
  if(verbose) {std::cout<<"[Kalman filter]: predict P_=\n"<<P_<<std::endl;}

}

void KalmanFilter::Update(const VectorXd &z, bool EKF) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Eigen::MatrixXd I = MatrixXd::Identity(4, 4);
  Eigen::VectorXd y = z; // use the dimension of z

  if(EKF)
  {
    Eigen::VectorXd observation = z; // use the dimension of z
    Observe(x_, observation);
    y = z - observation;
  }
  else
  {
      y = z - H_ * x_;
  }
  if(verbose) std::cout<<"[Kalman filter]: y=\n"<<y<<std::endl;
  Eigen::MatrixXd Ht = H_.transpose();
  if(verbose) std::cout<<"[Kalman filter]: Ht=\n"<<Ht<<std::endl;
  Eigen::MatrixXd S = H_ * P_ * Ht + R_;
  if(verbose) std::cout<<"[Kalman filter]: S=\n"<<S<<std::endl;
  Eigen::MatrixXd S_inv = S.inverse();
  if(verbose) std::cout<<"[Kalman filter]: S_inv=\n"<<S_inv<<std::endl;
  Eigen::MatrixXd K = P_ * Ht * S_inv;
  if(verbose) std::cout<<"[Kalman filter]: K=\n"<<K<<std::endl;
  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::Observe(const Eigen::VectorXd& state_vector,
                           Eigen::VectorXd& result) {
  double px = state_vector(0);
  double py = state_vector(1);
  double vx = state_vector(2);
  double vy = state_vector(3);

  double range = std::sqrt( px*px + py*py);
  double bearing = std::atan2(py, px);
  double range_rate;

  if(!Tools::isZero(range))
    range_rate = (px*vx + py*vy)/range;
  else
    range_rate = std::numeric_limits<double>::infinity();

  result<< range,
           bearing,
           range_rate;
}
