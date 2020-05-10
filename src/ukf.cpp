#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  std_a_ = 1.5;

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

   // Setting size of state
   n_x_ = 5;

   // Setting size of augemented vector
   n_aug_ = 7;

   // Setting value for design parameter, lambda
   lambda_ = 3 - n_x_;

   // Initialisaing Covariance matrix as Identity
   VectorXd p = VectorXd(n_x_);
   p << pow(std_radr_,2), pow(std_radr_,2), pow(std_radrd_,2),
        pow(std_radphi_,2), pow(std_radrd_,2);
   P_ = p.asDiagonal();



   // Initializing matrix for predicted sigma points
   Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

   // Setting size of weight vector
   weights_ = VectorXd(2*n_aug_+1);

   // Calculating and setting weight values
   weights_[0]  = lambda_/(lambda_ + n_aug_);

   Eigen::MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

   is_initialized_ = false;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   // */
   if(!is_initialized_) {
     if(meas_package.sensor_type_ == MeasurementPackage::LASER) {
       x_ << meas_package.raw_measurements_[0],
             meas_package.raw_measurements_[1],
             0,
             0,
             0;
     } else {
       double rho = meas_package.raw_measurements_[0];
       double phi = meas_package.raw_measurements_[1];
       double phi_dot = meas_package.raw_measurements_[2];

       //Polar to Cartesian coordinates
       x_ << rho*cos(phi),
            rho*sin(phi),
            0,
            0,
            0;
     }
     std::cout << "Initialized Unscented Kalman Filter" << std::endl;
     is_initialized_ = true;
     previous_timestamp = meas_package.timestamp_;
   }

   double delta_t = (meas_package.timestamp_ -  previous_timestamp)*1.0e-6;

   Prediction(delta_t);

   if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_==true)
    UpdateLidar(meas_package);
   else if (use_radar_==true)
     UpdateRadar(meas_package);
    previous_timestamp = meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
   VectorXd x_aug = VectorXd(n_aug_);
   x_aug << x_,
            0,
            0;

    Eigen::MatrixXd P_aug = MatrixXd(7, 7);
    P_aug.setZero();

    // Creating covariance matrix for augemented state
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5,5) = pow(std_a_, 2);
    P_aug(6,6) = pow(std_yawdd_, 2);
  // Calculating square root of matrix using Cholesky decomposition
   P_aug = P_aug.llt().matrixL();

   MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);;
   // Predicting sigma points
   Xsig_aug.col(0) = x_aug;
   for(int i =1; i <= n_aug_; i++) {
     Xsig_aug.col(i) = x_aug + sqrt(lambda_ + n_aug_)*P_aug.col(i-1);
     Xsig_aug.col(i+ n_aug_) = x_aug - sqrt(lambda_ + n_aug_)*P_aug.col(i-1);
   }

  // Process Model
   for(int i = 0; i < 2*n_aug_+1; i++) {
     VectorXd state = Xsig_aug.col(i);
     double px = state[0];
     double py = state[1];
     double v = state[2];
     double yaw = state[3];
     double yaw_rate = state[4];
     double a_var = state[5];
     double yaw_var = state[6];

     VectorXd process_noise(n_x_);
     if(yaw_rate==0){
       process_noise[0] =  v*cos(yaw)*delta_t + 0.5*pow(delta_t,2)*cos(yaw)*a_var;
       process_noise[1] =  v*sin(yaw)*delta_t + 0.5*pow(delta_t,2)*sin(yaw)*a_var;
     } else {
       process_noise[0] = (v/yaw_rate)*(sin(yaw+yaw_rate*delta_t)-sin(yaw));
       process_noise[1] = (v/yaw_rate)*(-cos(yaw+yaw_rate*delta_t)+cos(yaw));
     }
     process_noise[2] = delta_t*a_var;
     process_noise[3] = yaw_rate*delta_t + 0.5*pow(delta_t,2)*yaw_var;
     process_noise[4] = delta_t*yaw_var;

     state = state.head(5);

     state = state + process_noise;
     Xsig_pred_.col(i) = state;
   }

   VectorXd x_mean = VectorXd(n_x_);

   // Predicting state
   for(int i = 1; i < 2*n_aug_+1; i++) {
     weights_[i] = 1/(2*(lambda_ + n_aug_));
     x_mean += weights_[i]*Xsig_pred_.col(i);
   }

   // Predicting Covariance matrix
 for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
  VectorXd x_diff = Xsig_pred_.col(i) - x_mean;
  P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
   x_ = x_mean;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
   int n_z = 2;
   VectorXd z(2);
   MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

   // Defining vector for incoming LiDAR measurement
   z << meas_package.raw_measurements_[0],
        meas_package.raw_measurements_[1];

  VectorXd z_pred = VectorXd(n_z);

  // Calculating state for predicted measurement
  for (int i = 0; i < 2*n_aug_+1; i++) {
     VectorXd state = Xsig_pred_.col(i);
     Zsig.col(i) << state[0],
                    state[1];
     z_pred += weights_[i]*Zsig.col(i);
    }

    // Creating measurement noise matrix
    VectorXd r(2);
    r << pow(std_laspx_,2),
         pow(std_laspy_,2);
     MatrixXd R = r.asDiagonal();

    // Calculating Covaince matrix for predicted measurement
    MatrixXd S = MatrixXd(n_z,n_z);
    for (int i = 0; i < 2*n_aug_+1; i++) {
      VectorXd zdiff = Zsig.col(i) - z_pred;
      S += weights_[i]*zdiff*zdiff.transpose();
    }
    S = S + R;

    // Defining the cross-correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Calculating the cross-correlation matrix
  for(int i = 0; i<2*n_aug_+1; i++) {
    VectorXd zdiff = Zsig.col(i) - z_pred;
    VectorXd xdiff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i)*xdiff*zdiff.transpose();
    }

    // Calculating the Kalman Gain
    MatrixXd K = Tc*S.inverse();

    // Updating state and covaraince
    x_ += K*(z - z_pred);
    P_ +=  - K*S*K.transpose();


}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
   int n_z = 3;
   MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

   // Defining vector for incoming RADAR measurement
   VectorXd z(3);
   z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];

  VectorXd z_pred = VectorXd(n_z);
     // Calculating state for predicted measurement
   for (int i = 0; i < 2*n_aug_+1; i++) {
      VectorXd state = Xsig_pred_.col(i);
      double rho = sqrt(pow(state[0],2)+pow(state[1],2));
      double phi = atan(state[1]/state[0]);
      double rho_dot = (state[0]*state[2]*cos(state[3]) + state[1]*state[2]*sin(state[3]))/rho;
      Zsig.col(i) << rho,
                     phi,
                     rho_dot;
      z_pred += weights_[i]*Zsig.col(i);
    }

    // Calculating Covaince matrix for predicted measurement
    MatrixXd S = MatrixXd(n_z,n_z);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      VectorXd zdiff = Zsig.col(i) - z_pred;
      S += weights_[i]*zdiff*zdiff.transpose();
    }

    // Creating measurement noise matrix
    VectorXd r(3);
    r << pow(std_radr_,2),
      pow(std_radphi_,2),
      pow(std_radrd_,2);
     MatrixXd R = r.asDiagonal();

    S = S + R;

    // Defining the cross-correlation matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Calculating the cross-correlation matrix
  for(int i = 0; i<2*n_aug_+1; i++) {
    VectorXd zdiff = Zsig.col(i) - z_pred;
    VectorXd xdiff = Xsig_pred_.col(i) - x_;
    Tc += weights_(i)*xdiff*zdiff.transpose();
    }

    // Calculating the Kalman Gain
    MatrixXd K = Tc*S.inverse();

    // Updating state and covaraince
    x_ += K*(z - z_pred);
    P_ +=  - K*S*K.transpose();
}
