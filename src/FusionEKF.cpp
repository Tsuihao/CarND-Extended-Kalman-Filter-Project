#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include "config.h"
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
    Eigen::VectorXd x_init = VectorXd(4); // (Px, Py, Vx, Vy)
    x_init << 1, 1, 1, 1; // random init point
    ekf_.Set_x(x_init);

    Eigen::MatrixXd P_init = MatrixXd(4, 4);
    P_init << 100, 0 , 0, 0,
              0, 100, 0, 0,
              0,  0,  9, 0,
              0,  0,  0, 9;
    ekf_.Set_P(P_init);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // fully trust first the sensor measurement in initalize state
      cout << "[FusionEKF]: Radar: init" << endl;
      Hj_ = tools.CalculateJacobian(ekf_.Get_x());
      Eigen::VectorXd x_temp = Hj_.inverse() * measurement_pack.raw_measurements_;
      ekf_.Set_x(x_temp);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // fully trust first the sensor measurement in initalize state
      cout << "[FusionEKF]: LiDar: init" << endl;
      x_init(0) = measurement_pack.raw_measurements_(0); // re-write the Px of x_init
      x_init(1) = measurement_pack.raw_measurements_(1); // re-write the Py of x_init
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
  long long dt = t_curr - t_prev;

  std::cout<<"[FusionEKF]: Build Q and F matrix"<<std::endl;

  Eigen::MatrixXd Q = MatrixXd(4,4);
  Build_Q(Q, dt);

  Eigen::MatrixXd F = MatrixXd(4,4);
  Build_F(F, dt);

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
  bool useEKF = false;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    std::cout<<"[FusionEKF]: From Radar"<<std::endl;
    Hj_ = tools.CalculateJacobian(x_prev);
    ekf_.Set_update_params(Hj_, R_radar_);
    useEKF = true;
    ekf_.Update(measurement_pack.raw_measurements_, useEKF);

  } else {
    // Laser updates
    std::cout<<"[FusionEKF]: From Laser"<<std::endl;
    ekf_.Set_update_params(H_laser_, R_laser_);
    ekf_.Update(measurement_pack.raw_measurements_, useEKF);
  }
  std::cout<<"[FusionEKF]: KF Update [Completed]"<<std::endl;
  // print the output
  cout << "[FusionEKF]: x_ = " << ekf_.Get_x() << endl;
  cout << "[FusionEKF]: P_ = " << ekf_.Get_P() << endl;
}

void FusionEKF::Build_F(Eigen::MatrixXd& F, long long dt) const {

  F << 1, 0, dt,  0,
       0, 1,  0, dt,
       0, 0,  1,  0,
       0, 0,  0,  1;
}

void FusionEKF::Build_Q(Eigen::MatrixXd& Q, long long dt) const {

  double noise_ax = 9;
  double noise_ay = 9;

  long long dt_2 = dt * dt;
  long long dt_3 = dt_2 * dt;
  long long dt_4 = dt_2 * dt_2;

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
