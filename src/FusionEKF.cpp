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

    ekf_ = KalmanFilter();

    //create a 4D state vector, we don't know yet the values of the x state
    VectorXd x_in = VectorXd(4);
    x_in << 1, 1, 1, 1;

    //state covariance matrix P
    MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;

    //the initial transition matrix F_
    MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd Q_in = MatrixXd(4, 4);

    //ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);

    //ekf_ = KalmanFilter();
    ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);
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
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro, theta;
      // Don't need ro_dot;
      ro = measurement_pack.raw_measurements_[0];
      theta = measurement_pack.raw_measurements_[1];    // Phi?
      ekf_.x_ << ro * cos(theta), ro * sin(theta), 0, 0;

      //cout << "ro, theta = " << ro << " " << theta << endl;
      //cout << "x_ (init R) = " << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //cout << "px, py = " << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
      //cout << "x_ (init L) = " << ekf_.x_ << endl;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

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

   float noise_ax = 9;
   float noise_ay = 9;

    //cout << " Next meas = " << ekf_.x_ << endl;

   //compute the time elapsed between the current and previous measurements
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = measurement_pack.timestamp_;

//cout << " Predict 1: dt:" << dt << endl;

   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt;
   float dt_4 = dt_3 * dt;

//cout << " Predict 2: dt, dt_2, dt_3, dt_4" << dt << " " << dt_2 << " " << dt_3 << " " << dt_4 << endl;
   //Modify the F matrix so that the time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;

//cout << " Predict 3: dt, dt_2, dt_3" << dt << " " << dt_2 << " " << dt_3 << endl;   //set the process covariance matrix Q

   ekf_.Q_ = MatrixXd(4, 4);
   ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
              0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
              dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    //cout << " Predict Start" << endl;

    // Predict
    ekf_.Predict(); // x_, F_, P_, Q_ => Update: x_, P_

    //cout << " Predict Done" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    //cout << "x_ (R)= " << ekf_.x_ << endl;

    // Radar updates
    float rho, theta, rho_dot;
    // Don't need ro_dot;
    rho = measurement_pack.raw_measurements_[0];
    theta = measurement_pack.raw_measurements_[1];    // Phi?
    rho_dot = measurement_pack.raw_measurements_[2];    // Phi?
    //ekf_.x_ << ro * cos(theta), ro * sin(theta), 0, 0;

    VectorXd z = VectorXd(3);
    z << rho, theta, rho_dot;

    //cout << "z_ (R)= " << endl;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    //cout << " Jacobian() Done: {R}: z=" << z << endl;

    ekf_.UpdateEKF(z); // x_, P_, H_ R_, z; Update: x_, P_
    //cout << " UpdateEFK() Done: {R}" << endl;

  } else {

    // Laser updates
    VectorXd z = VectorXd(2);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(z);
    //cout << " Update() Done: {L}" << endl;
  }
  //cout << " Update All Done!" << endl;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
