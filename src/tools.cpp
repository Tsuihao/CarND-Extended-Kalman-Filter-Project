#include <iostream>
#include "tools.h"
#include <cmath> // sqrt, abs


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
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double q_2 = px*px + py*py;
  double q_root = std::sqrt(q_2);
  double q_3 = q2 * q_root;

  Eigen::MatrixXd J = MatrixXd(3,4);

  if(isZero(q2))
  {
    //TODO: Does this make sense?
    double inf = std::numeric_limites<double>::infinity();

    J << inf, inf, 0, 0,
         -inf, inf, 0, 0,
         inf, inf, inf, inf;
  }
  else
  {
    J << px/q_root, py/q_root, 0, 0,
         -py/q_2,   px/q_2,    0, 0,
         py*(vx*py - vy*px)/q_3,  px*(vy*px - vx*py)/q_3, px/q_root, py/q_root;

    return J;
  }
}

bool Tools::isZero(doulbe x)
{
  double EPS(std::numeric_limites<double>::epsilon());

  if(std::abs(x - 0.0) <= EPS * std::abs(x))
  {
    cout<< "It is zero!"<<endl;
    return true;
  }
  return false;
}
