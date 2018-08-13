#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Set the predict cycle parameters
   * @param x_in State vector
   * @param P_in State covariance
   * @param F_in Transition matrix
   * @param Q_in Process convariance
   */
  void Set_predict_params(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
                          Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);


   /**
    * Set the update cycle parameters
    * @param H_in Transformation matrix (state space -> measurement space)
    * @param R_in Measurement covariance
    */
  void Set_update_params(Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param EKF Flag to use EKF, the default is false;
   */
  void Update(const Eigen::VectorXd &z, bool EKF=false);

  /**
   * Set state vector
   */
  void Set_x(const Eigen::VectorXd &x_in);

  /**
   * Set state covariance
   */
  void Set_P(const Eigen::MatrixXd &P_in);

  /**
   * Get state vector, for tracing
   */
  const Eigen::VectorXd& Get_x() const;

  /**
   * Get state covariance, for tracing
   */
  const Eigen::MatrixXd& Get_P() const;


protected:
  /**
   * transfrom the state vector into measurement space.
   * @param state_vector The state vector x which need to be transformed
   *                     to measuremt space
   * @param measurment_space Result
   */
  void Observe(const Eigen::VectorXd& state_vector,
              Eigen::VectorXd& result);

private:
  Eigen::VectorXd x_; // state vector
  Eigen::MatrixXd P_; // state covariance matrix
  Eigen::MatrixXd F_; // state transition matrix
  Eigen::MatrixXd Q_; // process covariance matrix
  Eigen::MatrixXd H_; // measurement matrix
  Eigen::MatrixXd R_; // measurement covariance matrix


};

#endif /* KALMAN_FILTER_H_ */
