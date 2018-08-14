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
  VectorXd rmse(4);
  double inf = std::numeric_limits<double>::infinity();
  rmse<< inf, inf, inf, inf;
  if( estimations.size() == 0 || ground_truth.size() == 0)
  {
    std::cout<<"[tools]: invlaid input"<<std::endl;
    return rmse;
  }
  if( estimations.size() != ground_truth.size())
  {
    std::cout<<"[tools]: estimation and ground truth has different size"<<std::endl;
    return rmse;
  }

  rmse << 0, 0, 0, 0; // modify rmse from the 0;
  for(int i = 0; i < estimations.size(); ++i)
  {
    VectorXd temp = estimations[i] - ground_truth[i];
    VectorXd square = temp.array() * temp.array();
    rmse += square;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;

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
  double q_3 = q_2 * q_root;

  Eigen::MatrixXd J = MatrixXd(3,4);

  if(isZero(q_2))
  {
    //TODO: Does this make sense?
    double inf = std::numeric_limits<double>::infinity();

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

bool Tools::isZero(double x)
{
  double EPS(std::numeric_limits<double>::epsilon());

  if(std::abs(x - 0.0) <= EPS * std::abs(x))
  {
    cout<< "It is zero!"<<endl;
    return true;
  }
  return false;
}
