#include <iostream>
#include <cmath>
#include "kalman_filter.h"
#include "tools.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;

  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho, phi, rho_dot;
  //Tools tools;

    rho = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    if (fabs(x_[0]) > 0.0001) {
      phi = atan2(x_[1], x_[0]);
    } else {
        if (x_[0] * x_[1] > 0)
            phi = M_PI / 2;
        else if (x_[0] * x_[1] < 0)
            phi = -M_PI / 2;
    }
    if (phi > M_PI) {
        phi -= M_PI * 2;
    }
    if (phi < -M_PI) {
        phi += M_PI * 2;
    }

  if (fabs(rho) > 0.0001) {
      rho_dot = (x_[0] * x_[2] + x_[1] * x_[3]) / rho;
  } else {
      rho_dot = 1000000;
  }

  VectorXd Hx = VectorXd(3);
  Hx << rho, phi, rho_dot;

  // Update other Kalman matrix
  //H_ = tools.CalculateJacobian(x_);

  VectorXd yp = z - Hx;

  if (yp(1) > M_PI) {
      yp(1) -= M_PI * 2;
  }
  if (yp(1) < -M_PI) {
      yp(1) += M_PI * 2;
  }

  //cout << "z, Hx, yp: " << z << endl << endl << Hx << endl << endl << yp << endl;

  assert(fabs(yp(0)) < 3.0);
  //change coordination; yp: Polar coordination
  // Px, Py, Vx, Vy = ro * cos(theta), ro * sin(theta), 0, 0
  //VectorXd y = VectorXd(4);
  //y << yp(0) * cos(yp(1)), yp(0) * sin(yp(1)), 0, 0;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //cout << " Check 4: yp " << yp << endl << endl;
  //cout << " Check 4: x_, K, yp " << x_ << endl << K << endl << yp << endl;

  //new estimate
  x_ = x_ + (K * yp);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}
