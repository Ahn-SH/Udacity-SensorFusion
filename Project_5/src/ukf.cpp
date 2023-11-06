#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  is_initialized_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  n_x_ = 5;
  
  x_ = VectorXd::Zero(n_x_);

  // initial covariance matrix
  P_ = MatrixXd::Identity(n_x_,n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.0;
  
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
  time_us_ = 0.0;   

  n_aug_ = 7;  
  lambda_ = 3 - n_aug_;
  n_sig_ = 2 * n_aug_ + 1;  

  weights_ = VectorXd(n_sig_);       
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));

  weights_(0) = lambda_ / (lambda_ + n_aug_);

  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_);                     

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) 
    {
      x_(0) = meas_package.raw_measurements_[0];
      x_(1) = meas_package.raw_measurements_[1];
      
      P_(0,0) = std_laspx_ * std_laspx_;
      P_(1,1) = std_laspy_ * std_laspy_;
    }
    
    else if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
    {
     double rho = meas_package.raw_measurements_(0); 
     double phi = meas_package.raw_measurements_(1);
     double rho_dot = meas_package.raw_measurements_(2);
     
     
     double x = rho * cos(phi);
     double y = rho * sin(phi);
     double vx = rho_dot * cos(phi);
     double vy = rho_dot * sin(phi);
     double v = std::sqrt(vx * vx + vy * vy);
     
     x_ << x, y, v, rho, rho_dot;
     
     P_ << std_radr_* std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, std_radrd_ * std_radrd_, 0, 0,
            0, 0, 0, std_radphi_ * std_radphi_, 0,
            0, 0, 0, 0, std_radphi_ * std_radphi_;
    } 

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;      
    
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1e6;        
  time_us_ = meas_package.timestamp_;
  
  Prediction(dt); 

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) 
  {
    UpdateLidar(meas_package);
  } 
  else if (meas_package.sensor_type_ ==  MeasurementPackage::SensorType::RADAR) 
  {
    UpdateRadar(meas_package);
  }


}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  VectorXd x_aug = VectorXd::Zero(n_aug_);
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, n_sig_);

  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd L = P_aug.llt().matrixL();
  
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  
  for (int i = 0; i< n_sig_; ++i) {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    double px_pred, py_pred, v_pred, yaw_pred, yawd_pred;

    if (fabs(yawd) > 0.001) {
        px_pred = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_pred = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_pred = p_x + v*delta_t*cos(yaw);
        py_pred = p_y + v*delta_t*sin(yaw);
    }

    v_pred = v;
    yaw_pred = yaw + yawd*delta_t;
    yawd_pred = yawd;
    px_pred = px_pred + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_pred = py_pred + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_pred = v_pred + nu_a*delta_t;

    yaw_pred = yaw_pred + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_pred = yawd_pred + nu_yawdd*delta_t;

    Xsig_pred_(0,i) = px_pred;
    Xsig_pred_(1,i) = py_pred;
    Xsig_pred_(2,i) = v_pred;
    Xsig_pred_(3,i) = yaw_pred;
    Xsig_pred_(4,i) = yawd_pred;


  }  
  VectorXd x = VectorXd::Zero(n_x_);
  MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

  for (int i = 0; i < n_sig_; ++i) {  
    x = x + weights_(i) * Xsig_pred_.col(i);
  }

  for (int i = 0; i < n_sig_; ++i) {  
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }


  x_= x;
  P_ = P;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
   // Based on Kalman filter equation matrix
  
  MatrixXd H_ = MatrixXd(2, n_x_);     
  H_ << 1, 0, 0, 0, 0, 
        0, 1, 0, 0, 0; 
 
  MatrixXd R_ = MatrixXd(2, 2);			
  R_ << std_laspx_ * std_laspx_, 0,
        0, std_laspy_ * std_laspy_;
  
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());  
  
  VectorXd z_pred = H_ * x_;                          
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - z_pred;                         

  MatrixXd S = H_ * P_ * H_.transpose() + R_;     

  MatrixXd K = P_ * H_.transpose() * S.inverse();   

  x_ = x_ + (K * y);                             
  
  P_ = (I - K * H_) * P_;                
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  int n_z = 3;    
  

  MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig_);  
  VectorXd z_pred = VectorXd::Zero(n_z);        
  MatrixXd S = MatrixXd::Zero(n_z,n_z);         
  MatrixXd R = MatrixXd(n_z,n_z);   

  R << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;

  for (int i = 0; i <n_sig_; ++i) {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    
    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
    Zsig(1,i) = atan2(p_y,p_x);
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);
  }
  
  for (int i=0; i < n_sig_; ++i) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  for (int i = 0; i < n_sig_; ++i) { 
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  S = S + R;

  VectorXd z = meas_package.raw_measurements_; 
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  for (int i = 0; i < n_sig_; ++i) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd K = Tc * S.inverse();
  VectorXd z_diff = z - z_pred;

  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
}
