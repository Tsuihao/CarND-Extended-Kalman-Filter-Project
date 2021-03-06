#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_  << 1, 0, 0, 0,
               0, 1, 0, 0;

  // Hj_ can not be initailized due to lack of measuremnts.
  // X_
  Eigen::VectorXd x_init = VectorXd(4);
  x_init << 1, 1, 1, 1;
  ekf_.Set_x(x_init);

  // P_
  Eigen::MatrixXd P_init = MatrixXd(4, 4);
  P_init << 100, 0, 0, 0,
            0, 100, 0, 0,
            0, 0, 9, 0,
            0, 0, 0, 9;
  ekf_.Set_P(P_init);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "[FusionEKF]: initialize " << endl;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // fully trust first the sensor measurement in initalize state
      Hj_ = tools.CalculateJacobian(ekf_.Get_x());

      Eigen::VectorXd x_init = VectorXd(4); // (Px, Py, Vx, Vy);
      double rho = measurement_pack.raw_measurements_(0);
      double phi = measurement_pack.raw_measurements_(1);
      x_init<< rho * cos(phi), // Px
               rho * sin(phi), // Py
               0,
               0;
      ekf_.Set_x(x_init);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // fully trust first the sensor measurement in initalize state
      cout << "[FusionEKF]: LiDar: init" << endl;
      Eigen::VectorXd x_init = VectorXd(4); // (Px, Py, Vx, Vy);
      x_init<< measurement_pack.raw_measurements_(0), // Px
               measurement_pack.raw_measurements_(1), // Py
               0,
               0;
      ekf_.Set_x(x_init);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "[FusionEKF]: Init is completed" << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  std::cout<<"[FusionEKF]: KF Predict [Start]"<<std::endl;
  long long t_prev = previous_timestamp_;
  long long t_curr = measurement_pack.timestamp_;
  double dt = t_curr - t_prev;
  dt/=1000000; //ms to s
  previous_timestamp_ = t_curr; //update

  std::cout<<"[FusionEKF]: Build Q and F matrix, with dt="<<dt<<std::endl;

  Eigen::MatrixXd Q = MatrixXd(4,4);
  Build_Q(Q, dt);
  std::cout<<"[FusionEKF]: Q=\n"<<Q<<std::endl;
  Eigen::MatrixXd F = MatrixXd(4,4);
  Build_F(F, dt);
  std::cout<<"[FusionEKF]: F=\n"<<Q<<std::endl;

  // Set x_, P_, Q_, F_
  // Get the result from previous cycle
  Eigen::VectorXd x_prev = ekf_.Get_x();
  Eigen::MatrixXd P_prev = ekf_.Get_P();

  ekf_.Set_predict_params(x_prev, P_prev, F, Q);
  ekf_.Predict();

  std::cout<<"[FusionEKF]: KF Predict [Complete]"<<std::endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  std::cout<<"[FusionEKF]: KF Update [Start]"<<std::endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    std::cout<<"[FusionEKF]: From Radar, z=\n"<<measurement_pack.raw_measurements_<<std::endl;
    Hj_ = tools.CalculateJacobian(ekf_.Get_x());
    std::cout<<"[FusionEKF]: Hj_=\n"<<Hj_<<std::endl;
    ekf_.Set_update_params(Hj_, R_radar_);
    bool useEKF = true;
    ekf_.Update(measurement_pack.raw_measurements_, useEKF);

  } else {
    // Laser updates
    std::cout<<"[FusionEKF]: From Laser, z=\n"<<measurement_pack.raw_measurements_<<std::endl;
    ekf_.Set_update_params(H_laser_, R_laser_);
    bool useEKF = false;
    ekf_.Update(measurement_pack.raw_measurements_, useEKF);
  }
  std::cout<<"[FusionEKF]: KF Update [Completed]"<<std::endl;
  // print the output
  cout << "[FusionEKF]: x_ = \n" << ekf_.Get_x() << endl;
  cout << "[FusionEKF]: P_ = \n" << ekf_.Get_P() << endl;
}

void FusionEKF::Build_F(Eigen::MatrixXd& F, double dt) const {

  F << 1, 0, dt,  0,
       0, 1,  0, dt,
       0, 0,  1,  0,
       0, 0,  0,  1;
}

void FusionEKF::Build_Q(Eigen::MatrixXd& Q, double dt) const {

  double noise_ax = 9;
  double noise_ay = 9;

  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_2 * dt_2;

  double qx_1 = dt_4 * noise_ax / 4;
  double qx_2 = dt_3 * noise_ax / 2;
  double qx_3 = dt_2 * noise_ax;

  double qy_1 = dt_4 * noise_ay / 4;
  double qy_2 = dt_3 * noise_ay / 2;
  double qy_3 = dt_2 * noise_ay;

  Q << qx_1,   0,  qx_2,    0,
       0,   qy_1,     0, qy_2,
       qx_2,   0,  qx_3,    0,
       0,   qy_2,     0, qy_3;
}
