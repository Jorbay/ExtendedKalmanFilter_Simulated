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
  Tools tools;
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

  H_laser_ << 1,0,0,0,
	     0,1,0,0;

  Hj_ << 1,1,0,0,
	 1,1,0,0,
	 1,1,1,1;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //starting todo
  Eigen::MatrixXd F;
  F = MatrixXd(4,4);
  F << 1,0,1,0,
       0,1,0,1,
       0,0,1,0,
       0,0,0,1;
  ekf_.F_ = F; //blargh

  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
	    0,1,0,0,
	    0,0,1000,0,
	    0,0,0,1000;

  nx = 9;
  ny = 9;
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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    //is_initialized_ = true;
    //ekf_.Q_ =   


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float tempX = cos(phi)*rho;
      float tempY = sin(phi)*rho;
      ekf_.x_ << tempX, tempY, 0, 0; 
      	
      //VectorXd results = measurement_pack.raw_measurements_;
      //cout << results << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
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


  float elapsedTime = (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = elapsedTime*elapsedTime;
  float dt3 = dt2*elapsedTime;
  float dt4 = dt3*elapsedTime;

  ekf_.F_(0,2) = elapsedTime;
  ekf_.F_(1,3) = elapsedTime;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt4/4*nx, 0, dt3/2*nx, 0,
	    0, dt4/4*ny, 0, dt3/2*ny,
	    dt3/2*nx, 0, dt2*nx, 0,
	    0, dt3/2*ny, 0, dt2*ny;

  ekf_.Predict();




  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ =  tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); 
    
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
     
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;

  
}
