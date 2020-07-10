#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Helper function for debugging to detect when a matrix has at least one element that is NaN
bool isMatrixNan(MatrixXd input)
{
	for (int i = 0; i < input.rows(); i++)
	{
		for (int j = 0; j < input.rows(); j++)
		{
			if (std::isnan(input(i, j)))
			{
				return true;
			}
		}
	}
	return false;
}

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
  std_a_ = 3; //assuming that the max acceleration is 6m/s, so we take half the value /*30*/;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1; // Assuming max of 2 rad/sec2
  
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
   * Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  n_z_radar_ = 3;
  n_z_laser_ = 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // Initialize weights
  weights_ = VectorXd(2*n_aug_+1);
  double otherWeights = 0.5/(lambda_+n_aug_);
  weights_(0) = lambda_/(lambda_+n_aug_);

  for (int i=1; i<2*n_aug_+1; ++i) {
    weights_(i) = otherWeights;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

   // Check if initialization was done or not
   if (!is_initialized_) {
       //std::cout << "Kalman Filter Initialization " << std::endl;

       double px = 0;
       double py = 0;
       double v = 0;
       double psi = 0;
       double psi_dot = 0;

       // Use Identiy matrix except for V and yaw (to correct errors in Vy calculation)
       P_ <<   1,   	0,   	0,   	0,   	0,
    		   0,   	1,   	0,   	0,   	0,
			   0,   	0,   	1000, 	0,   	0,
			   0,   	0,   	0,   	0.0225, 0,
			   0,   	0,   	0,   	0,   	1;

       // set the state with the initial location and zero velocity
       if (meas_package.sensor_type_ == MeasurementPackage::LASER)
       {
          // Initialize the state with the reading of laser
          px = meas_package.raw_measurements_[0];
          py = meas_package.raw_measurements_[1];

       }
       else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
       {
          // Initialize the state with the reading of radar

          // Extract radar measurement
          double distance = meas_package.raw_measurements_[0];
          double angle    = meas_package.raw_measurements_[1];
          double radial_velocity = meas_package.raw_measurements_[2];

          // Compute px
          px = distance * cos(angle);

          // Compute py
           py = distance * sin(angle);

           v = radial_velocity;

           // Overwrite P for the velocity
//           P_(2,2) = 1;
       }

       x_ << px, py, v, psi, psi_dot;

       time_us_ = meas_package.timestamp_;
       is_initialized_ = true;
       return;
     }

     // compute the time elapsed between the current and previous measurements
     // dt - expressed in seconds
     double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;

     if (dt == 0.0)
     {
    	 //std::cout << "DELTA T IS ZERO" << std::endl;
    	 //std::cout << "  time_us_: " << time_us_ << std::endl;
    	 //std::cout << "  meas_package.timestamp_: " << meas_package.timestamp_ << std::endl;
     }

     // Update the stored timestamp in the object
     time_us_ = meas_package.timestamp_;

     // 3. Call the Kalman Filter predict() function
     Prediction(dt);

     // Update measurement according to the sensor type
     // Pass it the measurement pack
     if (meas_package.sensor_type_ == MeasurementPackage::LASER)
     {
        // Transformed sigment points
        MatrixXd Zsig = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);
        Zsig.fill(0);

        // mean predicted measurement
        VectorXd z_pred = VectorXd(n_z_laser_);
        z_pred.fill(0);

        // measurement covariance matrix S
        MatrixXd S = MatrixXd(n_z_laser_, n_z_laser_);
        S.fill(0);

        // Update Radar matrices
        UpdateLaserMatrices(Zsig, z_pred, S);

        // Update state based on Radar measurement
        UpdateLaserState(Zsig, z_pred, S, meas_package);
     }
     else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
     {
        // Transformed sigment points
        MatrixXd Zsig = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
        Zsig.fill(0);

        // mean predicted measurement
        VectorXd z_pred = VectorXd(n_z_radar_);
        z_pred.fill(0);

        // measurement covariance matrix S
        MatrixXd S = MatrixXd(n_z_radar_, n_z_radar_);
        S.fill(0);

        // Update Radar matrices
        UpdateRadarMatrices(Zsig, z_pred, S);

        // Update state based on Radar measurement
        UpdateRadarState(Zsig, z_pred, S, meas_package);
     }
     else
     {
        std::cerr << "Error: Wrong sensor type!!!" << std::endl;
     }
}

void UKF::Prediction(double delta_t) {
  /**
   * Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

   // Generate augmented sigma points
   MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
   Xsig_aug.fill(0);
   GenerateAugmentedSigmaPoints(Xsig_aug);

   //std::cout << "Done: Generated Augmented Sigma points" << std::endl;

   // Predict Sigma Points using augmented sigma points
   Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
   Xsig_pred_.fill(0);
   SigmaPointPrediction(Xsig_aug, delta_t);

   //std::cout << "Done: Sigma Points Prediction" << std::endl;

   // 2. Predict mean and covariance matrix
   PredictMeanAndCovariance();

   //std::cout << "Predict mean and cov" << std::endl;
}


void UKF::GenerateAugmentedSigmaPoints(MatrixXd &Xsig_aug_out) {
   /*
   P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
         -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
          0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
         -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
         -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
         */

   // create augmented mean vector
   VectorXd x_aug = VectorXd(n_aug_);

   // create augmented state covariance
   MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
   P_aug.fill(0); // initialize with zeros

   // create augmented mean state
   x_aug.head(5) = x_;
   x_aug(5) = 0;
   x_aug(6) = 0;

   P_aug.topLeftCorner(n_x_,n_x_) = P_;
   P_aug(5,5) = std_a_ * std_a_;
   P_aug(6,6) = std_yawdd_ * std_yawdd_;

   //std::cout << "P_aug = " << std::endl << P_aug << std::endl;

   // create square root matrix
   MatrixXd L = P_aug.llt().matrixL();

   // std::cout << "P_aug = " << std::endl << P_aug << std::endl;

   //std::cout << "L = " << std::endl << L << std::endl;

   // create augmented sigma points
   // set first column of sigma point matrix
   Xsig_aug_out.col(0) = x_aug;

   // set remaining sigma points
   for (int i = 0; i < n_aug_; ++i) {
      Xsig_aug_out.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
      Xsig_aug_out.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
   }

   // print result
   //std::cout << "Xsig_aug = " << std::endl << Xsig_aug_out << std::endl;
}


void UKF::SigmaPointPrediction(MatrixXd& Xsig_aug, double delta_t) {

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
     //std::cout << " Inside loop: " << i << std::endl;

      VectorXd col1 = VectorXd(n_x_);
      VectorXd col2 = VectorXd(n_x_);

      double px      = Xsig_aug(0, i);
      double py      = Xsig_aug(1, i);
      double v       = Xsig_aug(2, i);
      double psi     = Xsig_aug(3, i);
      double psi_dot = Xsig_aug(4, i);
      double neu_a   = Xsig_aug(5, i);
      double neu_psi = Xsig_aug(6, i);

      //if (psi_dot != 0) Avoid division by zero by checking that psi_dot is not too small
      if (fabs(psi_dot) > 0.001)
      {
    	  col1(0) = v / psi_dot * (  sin(psi + psi_dot * delta_t) - sin(psi) );
          col1(1) = v / psi_dot * ( -cos(psi + psi_dot * delta_t) + cos(psi) );
      }
      else
      {
    	  col1(0) = v * cos(psi) * delta_t;
          col1(1) = v * sin(psi) * delta_t;
      }

      col1(2) = 0;
      col1(3) = psi_dot * delta_t;
      col1(4) = 0;

      col2(0) = 0.5 * delta_t * delta_t * cos(psi) * neu_a;
      col2(1) = 0.5 * delta_t * delta_t * sin(psi) * neu_a;
      col2(2) = delta_t * neu_a;
      col2(3) = 0.5 * delta_t * delta_t * neu_psi;
      col2(4) = delta_t * neu_psi;

      //std::cout << "  before writing back to matrix" << std::endl;
      //std::cout << "Col1: " << col1 << std::endl;
      //std::cout << "Col2: " << col2 << std::endl;

      Xsig_pred_.col(i) = Xsig_aug.col(i).head(5) + col1 + col2;
  }

  // write predicted sigma points into right column


  // print result
  //std::cout << "Detla t = " << delta_t << std::endl;
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;
}


void UKF::PredictMeanAndCovariance() {

   // predict state mean
   VectorXd predMean = VectorXd(n_x_);
   predMean.fill(0);

   //std::cout<< "Xsig_pred_ \n" << Xsig_pred_ << std::endl;

   //std::cout << "Weights " << weights_ << std::endl;


   for (int i =0; i < weights_.size(); i++)
   {
      predMean += weights_(i) * Xsig_pred_.col(i);
   }

   //std::cout << "        predMean(0)" << predMean(0) << std::endl;

   // Update state vector with the predicted mean
   x_ = predMean;

   // predict state covariance matrix
   MatrixXd predCov = MatrixXd(n_x_, n_x_);
   predCov.fill(0);

   for (int i =0; i < weights_.size(); i++)
   {
      // Create error matrix to hold the difference between the predicted signa points
      // and the mean
      VectorXd xDiff = VectorXd(n_x_);

      xDiff = Xsig_pred_.col(i) - x_;

      // Angle normalization
      while (xDiff(3) > M_PI) xDiff(3)-=2.*M_PI;
      while (xDiff(3) <-M_PI) xDiff(3)+=2.*M_PI;

      predCov += weights_(i) * xDiff * xDiff.transpose();
   }
   P_ = predCov;

   // print result
   //std::cout << "Predicted state" << std::endl;
   //std::cout << x_ << std::endl;
   //std::cout << "Predicted covariance matrix" << std::endl;
   //std::cout << P_ << std::endl;
}

void UKF::UpdateRadarMatrices(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S ) {
  /**
   * Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

   // 1. Transform sigma points into measurement space in Zsig
   for (int i = 0; i < 2 * n_aug_ + 1; i++)
   {
     double px 	= Xsig_pred_(0, i);
     double py  = Xsig_pred_(1, i);
     double v   = Xsig_pred_(2, i);
     double psi = Xsig_pred_(3, i);
     //double psi_dot = Xsig_pred_(4, i);

     // Radial distance
     Zsig(0, i) = sqrt(px*px + py*py);

     // Angle
     Zsig(1, i) = atan2(py,px);

     // Radial Velocity
     Zsig(2, i) = (px * v * cos(psi) + py * v* sin(psi)) / Zsig(0, i);
   }

   // 2. Calculate mean predicted measurement
   for (int i = 0; i < weights_.size(); i++)
   {
     z_pred += weights_(i) * Zsig.col(i);
   }

   // 3. calculate innovation covariance matrix S
   for (int i = 0; i < weights_.size(); i++)
   {
     VectorXd zDiff = VectorXd(5);

     zDiff = Zsig.col(i) - z_pred;

     // Angle normalization
     while (zDiff(1) > M_PI)	zDiff(1)-=2.*M_PI;
     while (zDiff(1) <-M_PI) 	zDiff(1)+=2.*M_PI;

     S += weights_(i) * zDiff * zDiff.transpose();
   }

   // Add R
   MatrixXd R = MatrixXd(3,3);

   R << std_radr_ * std_radr_,      0 ,                          0,
      0,                            std_radphi_ * std_radphi_,   0,
      0,                            0,                           std_radrd_ * std_radrd_;

   S = S + R;

   // print result
   //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
   //std::cout << "S: " << std::endl << S << std::endl;
}



void UKF::UpdateRadarState(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S,  MeasurementPackage& meas_pack) {

   // create vector for incoming radar measurement
   VectorXd z = meas_pack.raw_measurements_;

   // create matrix for cross correlation Tc
   MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);
   Tc.fill(0);

   // calculate cross correlation matrix Tc(n_x,n_z)
   for (int i =0; i < 2 * n_aug_ + 1; i++)
   {
      VectorXd xDiff = VectorXd(n_x_);
      VectorXd zDiff = VectorXd(n_z_radar_);

      xDiff = Xsig_pred_.col(i) - x_;

      // angle normalization
      while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
      while (xDiff(3)<-M_PI) xDiff(3)+=2.*M_PI;

      zDiff = Zsig.col(i) - z_pred;

      // angle normalization
      while (zDiff(1)> M_PI) zDiff(1)-=2.*M_PI;
      while (zDiff(1)<-M_PI) zDiff(1)+=2.*M_PI;

      Tc += weights_(i) * xDiff * zDiff.transpose();
   }

   // calculate Kalman gain K;
   MatrixXd K = MatrixXd(n_x_, n_z_radar_);
   K.fill(0);

   if (S.determinant() == 0)
   {
	   std::cout << "=========== S is Singular ============" <<std::endl;

	   std::cout << S << std::endl;
   }

   K = Tc * S.inverse();
#if 1
   //std::cout << "\nPredicted Px            = " << x_(0) << std::endl;
   //std::cout <<   "Measured Px radar       = " << z(0) * cos(z(1)) << std::endl;
   //std::cout <<   "Mean Predicted Px radar = " << z_pred(0) * cos(z_pred(1)) << std::endl;

   //std::cout << "\nPredicted Py            = " << x_(1) << std::endl;
   //std::cout <<   "Measured Py radar       = " << z(0) * sin(z(1)) << std::endl;
   //std::cout <<   "Mean Predicted Py radar = " << z_pred(0) * sin(z_pred(1)) << std::endl;

   //std::cout << "\nPredicted V            = " << x_(2) << std::endl;
   //std::cout <<   "Measured V radar       = " << z(1)  << std::endl;
   //std::cout <<   "Mean Predicted V radar = " << z_pred(1) * sin(z_pred(1)) << std::endl;

#endif

   // update state mean and covariance matrix
   VectorXd newZDiff = (z - z_pred);
   // angle normalization
   while (newZDiff(1)> M_PI)	newZDiff(1) -= 2.*M_PI;
   while (newZDiff(1)<-M_PI)	newZDiff(1) += 2.*M_PI;

   x_ = x_ + K * newZDiff;

#if 1
   //std::cout << "|S|                     = " << S.determinant() << std::endl;
   //std::cout << " >> New Px              = " << x_(0) << std::endl;
   //std::cout << " >> New Py              = " << x_(1) << std::endl;
   //std::cout << " >> New V Radar         = " << x_(2) << std::endl;
   //std::cout << " >> New psi Radar         = " << x_(3)*180 << std::endl;
#endif

#if 0
   if (std::isnan(x_(0)))
   {
	   std::cout << "=========== Px is NaN ==========" << std::endl;
	   std::cout << "K" << K << std::endl;
	   std::cout << "Tc" << Tc << std::endl;
	   std::cout << "S.inverse()" << S.inverse() << std::endl;
	   std::cout << "Xsig_pred_" << Xsig_pred_ << std::endl;
	   std::cout << "Zsig" << Zsig << std::endl;
   }
#endif

   P_ = P_ - K * S * K.transpose();

   //std::cout << "Updated K radar: " << std::endl << K << std::endl;

   // print result
   //std::cout << "Updated state radar x: " << std::endl << x_ << std::endl;
   if (isMatrixNan(P_))
   {
	   std::cout << "Updated state radar covariance P: " << std::endl << P_ << std::endl;
   }

   // Calculate NIS
   double NIS = newZDiff.transpose() * S.inverse() * newZDiff;
   //std::cout << "NIS Radar," << NIS << std::endl;
}


void UKF::UpdateLaserMatrices(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

   // 1. Transform sigma points into measurement space in Zsig
   for (int i = 0; i < 2 * n_aug_ + 1; i++)
   {
     double px = Xsig_pred_(0, i);
     double py = Xsig_pred_(1, i);
     //double v  = Xsig_pred_(2, i);
     //double psi = Xsig_pred_(3, i);
     //double psi_dot = Xsig_pred_(4, i);

     // px
     Zsig(0, i) = px;

     // py
     Zsig(1, i) = py;
   }

   // 2. Calculate mean predicted measurement
   for (int i = 0; i < weights_.size(); i++)
   {
     z_pred += weights_(i) * Zsig.col(i);
   }

   // 3. calculate innovation covariance matrix S
   for (int i = 0; i < weights_.size(); i++)
   {
     VectorXd errorVector = VectorXd(5);

     errorVector = Zsig.col(i) - z_pred;

     S += weights_(i) * errorVector * errorVector.transpose();
   }

   // Add R
   MatrixXd R = MatrixXd(2,2);

   R <<    std_laspx_ * std_laspx_,   0 ,
         0,                      std_laspy_ * std_laspy_;


   S = S + R;

   // print result
   //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
   //std::cout << "S: " << std::endl << S << std::endl;
}


void UKF::UpdateLaserState(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S,  MeasurementPackage& meas_pack) {

   // create example vector for incoming radar measurement
   VectorXd z = meas_pack.raw_measurements_;

   // create matrix for cross correlation Tc
   MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);
   Tc.fill(0);

   // calculate cross correlation matrix Tc(n_x,n_z)
   for (int i =0; i < 2 * n_aug_ + 1; i++)
   {
      VectorXd xDiff = VectorXd(n_x_);
      VectorXd zDiff = VectorXd(n_z_laser_);

      xDiff = Xsig_pred_.col(i) - x_;

      // angle normalization
      while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
      while (xDiff(3)<-M_PI) xDiff(3)+=2.*M_PI;

      zDiff = Zsig.col(i) - z_pred;

      Tc += weights_(i) * xDiff * zDiff.transpose();
   }


   // calculate Kalman gain K;
   MatrixXd K = MatrixXd(n_x_, n_z_laser_);
   K.fill(0);

   if (S.determinant() == 0)
   {
	   std::cout << "=========== S is Singular ============" <<std::endl;

	   std::cout << S << std::endl;
   }

   K = Tc * S.inverse();

#if 1
   //std::cout << "\nPredicted Px            = " << x_(0) << std::endl;
   //std::cout <<   "Measured Px laser       = " << z(0)  << std::endl;
   //std::cout <<   "Mean Predicted Px laser = " << z_pred(0) << std::endl;

   //std::cout << "\nPredicted Py            = " << x_(1) << std::endl;
   //std::cout <<   "Measured Py laser       = " << z(1)  << std::endl;
   //std::cout <<   "Mean Predicted Py laser = " << z_pred(1) << std::endl;

   //std::cout << "\nPredicted Vy            = " << x_(2) * sin(x(3)) << std::endl;
   //std::cout <<   "Measured Vy laser       = " << z(0)  << std::endl;
   //std::cout <<   "Mean Predicted Vy laser = " << z_pred(0) << std::endl;
#endif

   // update state mean and covariance matrix
   VectorXd diff = (z - z_pred);
   x_ = x_ + K * diff;

#if 1
   //std::cout <<   "|S|                     = " << S.determinant() << std::endl;
   //std::cout << " >> New Px                  = " << x_(0) << std::endl;
   //std::cout << " >> New Py                  = " << x_(1) << std::endl;
   //std::cout << " >> New V Laser         = " << x_(2) << std::endl;
   //std::cout << " >> New Psi Laser         = " << x_(3)*180 << std::endl;
#endif

   P_ = P_ - K * S * K.transpose();

   // Calculate NIS
   double NIS = diff.transpose() * S.inverse() * diff;
   //std::cout << "NIS Laser," << NIS << std::endl;

   // print result
   //std::cout << "Updated state laser x: " << std::endl << x_ << std::endl;
   if (isMatrixNan(P_))
   {
	   //std::cout << "Updated state laser covariance P: " << std::endl << P_ << std::endl;

	   //std::cout << "Tc " << Tc << std::endl;
	   //std::cout << "K " << K << std::endl;
	   //std::cout << "S " << K << std::endl;
	   //std::cout << "K Transpose " << K.transpose() << std::endl;
   }
}


