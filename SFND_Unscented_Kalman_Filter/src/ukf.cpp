#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // Initialized?
  is_initialized_ = false;

  // Previous timestamp
  time_us_ = 0;

  // State dimension
  n_x_ = 5;

  // Augmented dimension
  n_aug_ = 7;

  // Spreading parameter
  lambda_ = 3 - n_aug_;

  // Number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Set weight vector
  weights_ = VectorXd(n_sig_);
  weights_(0.0) = lambda_ / (lambda_ + n_aug_);
  double weight = 0.5 / (lambda_ + n_aug_);
  for (int i=1; i<n_sig_; i++){
      weights_(i) = weight;
  }

  // Create matrix for predicted sigma points in state space
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // Initialize state space
  x_.fill(0.5);

  // Initialize state covariance matrix
  P_.fill(0.0);
  P_.diagonal() << 0.75, 0.75, 0.75, 0.75, 0.75;

  // Set lidar measurement covariance matrix
  n_lidar_ = 2;
  R_lidar_ = MatrixXd(n_lidar_, n_lidar_);
  R_lidar_.fill(0.0);
  R_lidar_.diagonal() << std_laspx_*std_laspx_, std_laspy_*std_laspy_;

  // Set radar measurement covariance matrix
  n_radar_ = 3;
  R_radar_ = MatrixXd(n_radar_, n_radar_);
  R_radar_.fill(0.0);
  R_radar_.diagonal() << std_radr_*std_radr_, std_radphi_*std_radphi_, std_radrd_*std_radrd_;

  // Set process noise covariance matrix
  Q_ = MatrixXd(n_aug_-n_x_, n_aug_-n_x_);
  Q_.fill(0.0);
  Q_.diagonal() << std_a_*std_a_, std_yawdd_*std_yawdd_;

  // Set print options
  print_lidar = false;
  print_radar = false;

  // Use radar velocity component?
  use_radar_v = false;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  //// 0. Toggle between using either lidar or radar
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_){
      return;
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_){
      return;
  }

  //// 1. Initialize state
  if (! is_initialized_){
      if (meas_package.sensor_type_ == MeasurementPackage::LASER){
          x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;
      } else {
          double rho = meas_package.raw_measurements_(0);
          double phi = meas_package.raw_measurements_(1);
          double rhod = meas_package.raw_measurements_(2);
          double v = 0;

          if (use_radar_v){
              // Calculation of v component of state vector.
              // NOTE: This is a close approximation to velocity component using trigonometric relations
              double v_x = rhod * cos(phi);
              double v_y = rhod * sin(phi);
              v = sqrt(v_x*v_x + v_y*v_y);
          }

          // x_ << rho * sin(phi), rho * cos(phi), v, 0, 0;
          x_ << rho * cos(phi), rho * sin(phi), v, 0, 0;
      }

      time_us_ = meas_package.timestamp_;

      is_initialized_ = true;

      return;
  }

  //// 2. Predict
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  //// 3. Update measurements
  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      UpdateLidar(meas_package);
  } else {
      UpdateRadar(meas_package);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //// 1. Generate augmented sigma points

    // Create augmented sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    Xsig_aug.fill(0.0);

    // Create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0);
    x_aug.head(n_x_) = x_;

    // Create augmented state covariance matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.bottomRightCorner(Q_.rows(), Q_.cols()) = Q_;

    // Compute square root of augmented state covariance matrix using Cholesky decomposition
    MatrixXd A = P_aug.llt().matrixL();
    double factor = sqrt(lambda_ + n_aug_);

    // Better vectorization
    Xsig_aug.col(0) = x_aug;
    Xsig_aug.block(0, 1, n_aug_, n_aug_) = (factor * A).colwise() + x_aug;
    Xsig_aug.block(0, n_aug_+1, n_aug_, n_aug_) = (-A * factor).colwise() + x_aug;

    // Broadcast x_aug but still using for loop
    //    MatrixXd X_aug = MatrixXd(n_aug_, n_aug_);
    //    for (int j=0; j<n_aug_; j++){
    //        X_aug.col(j) = x_aug;
    //    }
    //    Xsig_aug.col(0) = x_aug;
    //    Xsig_aug.block(0, 1, n_aug_, n_aug_) = X_aug + factor * A;
    //    Xsig_aug.block(0, n_aug_ + 1, n_aug_, n_aug_) = X_aug - factor * A;

    //// For loop approach from assignment solution
    //    Xsig_aug.col(0) = x_aug;
    //    for (int j=0; j<n_aug_; j++){
    //        Xsig_aug.col(j + 1) = x_aug + factor * A.col(j);
    //        Xsig_aug.col(j + 1 + n_aug_) = x_aug - factor * A.col(j);
    //    }

    //// This vectorization does't give same result as assignment for loop!
    // TODO: Debug
    //    Xsig_aug.col(0) = x_aug;
    //    Xsig_aug.block(0, 1, n_aug_, n_aug_) = (factor * A).colwise() + x_aug;
    //    Xsig_aug.block(0, n_aug_+1, n_aug_, n_aug_) = (-A * factor).colwise() + x_aug;

    //// This vectorization also does't give same result as assignment for loop!
    // TODO: Debug
    //    A = A * sqrt(lambda_ + n_aug_);
    //    A.colwise() += x_aug;
    //
    //    Xsig_aug.col(0) = x_aug;
    //    Xsig_aug.block(0, 1, n_aug_, n_aug_) = A;
    //
    //    A.colwise() -= x_aug;
    //    A = -1 * A;
    //    A.colwise() += x_aug;
    //    Xsig_aug.block(0, n_aug_+1, n_aug_, n_aug_) = A;

  //// 2. Predict sigma points

    for (int j=0; j<n_sig_; j++){

        // Extract values
        double v = Xsig_aug(2, j);
        double yaw = Xsig_aug(3, j);
        double yawd = Xsig_aug(4, j);
        double nu_a = Xsig_aug(5, j);
        double nu_yawdd = Xsig_aug(6, j);

        // Deterministic part: x_k + f(x_k)
        VectorXd temp = VectorXd(n_x_);
        temp = Xsig_aug.block(0, j, n_x_, 1);

        // Deterministic part: Integral solution and avoiding division by zero
        if (fabs(yawd) > 0.001){
            temp(0) += v * (sin(yaw + yawd * delta_t) - sin(yaw)) / yawd;
            temp(1) += v * (-cos(yaw + yawd * delta_t) + cos(yaw)) / yawd;
            temp(3) += yawd * delta_t;
        } else {
            temp(0) += v * cos(yaw) * delta_t;
            temp(1) += v * sin(yaw) * delta_t;
        }

        // Noise part
        temp(0) += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
        temp(1) += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
        temp(2) += delta_t * nu_a;
        temp(3) += 0.5 * delta_t * delta_t * nu_yawdd;
        temp(4) += delta_t * nu_yawdd;

        // Write predicted sigma points
        Xsig_pred_.block(0, j, n_x_, 1) = temp;
    }

  //// 3. Predict mean state and covariance matrix
    // Predict mean state
    x_.fill(0.0);
    for (int j=0; j<n_sig_; j++){
        x_ = x_ + weights_(j) * Xsig_pred_.col(j);
    }

    // Predict covariance matrix
    P_.fill(0.0);
    for (int j=0; j<n_sig_; j++){

        // State difference
        VectorXd x_diff = Xsig_pred_.col(j) - x_;

        // Angle normalization
        NormalizeAngle(x_diff, 3);

        P_ = P_ + weights_(j) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  //// 1. Predict measurement

    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_lidar_, n_sig_);

    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_lidar_);

    // Measurement covariance matrix S
    MatrixXd S = MatrixXd(n_lidar_, n_lidar_);

    // Transform sigma points into measurement space
    Zsig = Xsig_pred_.block(0, 0, n_lidar_, n_sig_);

    // Calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int j=0; j<n_sig_; j++){
        z_pred = z_pred + weights_(j) * Zsig.col(j);
    }

    // Calculate measurement covariance matrix S
    S.fill(0.0);
    for (int j=0; j<n_sig_; j++){

        // Measurement residual
        VectorXd z_diff = Zsig.col(j) - z_pred;

        S = S + weights_(j) * z_diff * z_diff.transpose();
    }

    S = S + R_lidar_;

  //// 2. Update state and covariance
    // Create cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_lidar_);

    // Calculate cross correlation
    Tc.fill(0.0);
    for (int j=0; j<n_sig_; j++){
        // State and measurement residuals
        VectorXd x_diff = Xsig_pred_.col(j) - x_;
        VectorXd z_diff = Zsig.col(j) - z_pred;

        // Normalize angle value in state residual
        NormalizeAngle(x_diff, 3);

        Tc = Tc + weights_(j) * x_diff * z_diff.transpose();
    }

    // Calculate Kalman gain
    MatrixXd K = Tc * S.inverse();

    // Measurement residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    // Update x_ and P_
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    if (print_lidar){
        cout << "Lidar x_ \n" << x_ << endl;
        cout << endl;
        cout << "Lidar P_ \n" << P_ << endl;
        cout << endl;
    }

    // NIS
    double epsilon = z_diff.transpose() * S.inverse() * z_diff;
    if (epsilon > 5.991){
        cout << "Lidar NIS: " << epsilon << " > 5.991 => Exceeding Threshold!" << endl;
    }

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  //// 1. Predict measurement
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_radar_, n_sig_);

    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_radar_);

    // Measurement covariance matrix S
    MatrixXd S = MatrixXd(n_radar_, n_radar_);

    // Transform sigma points into measurement space
    for (int j=0; j<n_sig_; j++){
        // Extract values for calculation
        double p_x = Xsig_pred_(0, j);
        double p_y = Xsig_pred_(1, j);
        double v = Xsig_pred_(2, j);
        double yaw = Xsig_pred_(3, j);

        // Apply measurement model to transform sigma points in measurement space
        Zsig(0, j) = sqrt(p_x*p_x + p_y*p_y);
        Zsig(1, j) = atan2(p_y, p_x);
        Zsig(2, j) = (p_x * v * cos(yaw) + p_y * v * sin(yaw)) / sqrt(p_x*p_x + p_y*p_y);
    }

    // Calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int j=0; j<n_sig_; j++){
        z_pred = z_pred + weights_(j) * Zsig.col(j);
    }

    // Calculate measurement covariance matrix S
    S.fill(0.0);
    for (int j=0; j<n_sig_; j++){

        // Measurement residual
        VectorXd z_diff = Zsig.col(j) - z_pred;

        // Angle normalization
        NormalizeAngle(z_diff, 1);

        S = S + weights_(j) * z_diff * z_diff.transpose();
    }

    S = S + R_radar_;

  //// 2. Update state and covariance
    // Create cross correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_radar_);

    // Calculate cross correlation
    Tc.fill(0.0);
    for (int j=0; j<n_sig_; j++){
        // State and measurement residuals
        VectorXd x_diff = Xsig_pred_.col(j) - x_;
        VectorXd z_diff = Zsig.col(j) - z_pred;

        // Normalize angle value in state residual
        NormalizeAngle(x_diff, 3);
        NormalizeAngle(z_diff, 1);

        Tc = Tc + weights_(j) * x_diff * z_diff.transpose();
    }

    // Calculate Kalman gain
    MatrixXd K = Tc * S.inverse();

    // Measurement residual
    VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

    // Normalize angle
    NormalizeAngle(z_diff, 1);

    // Update x_ and P_
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();


    if (print_radar){
        cout << "Radar x_ \n" << x_ << endl;
        cout << endl;
        cout << "Radar P_ \n" << P_ << endl;
        cout << endl;
    }

    // NIS
    double epsilon = z_diff.transpose() * S.inverse() * z_diff;
    if (epsilon > 7.815){
        cout << "Radar NIS: " << epsilon << " > 7.815 => Exceeding Threshold!" << endl;
    }
}

void UKF::NormalizeAngle(VectorXd &vec, int idx) {
    // Angle normalization:  state contains an angle, subtracting angles is a problem
    // for Kalman filters, because the result might be 2π plus a small angle, instead
    // of just a small angle so normalize the angle
    while (vec(idx) > M_PI){
        vec(idx) -= 2 * M_PI;
    }
    while (vec(idx) < -M_PI){
        vec(idx) += 2 * M_PI;
    }
}