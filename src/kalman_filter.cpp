#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    /**
     TODO:
     * predict the state
     */

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
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
    /**
     TODO:
     * update the state by using Kalman Filter equations
     */
}

MatrixXd GetRadarMeas(const VectorXd &x_state)
{
    VectorXd z_radar;
    z_radar = VectorXd(3);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    float min_r = 0.0001;
    z_radar(0) = sqrt(pow(px, 2) + pow(py, 2));
    if (z_radar(0)< min_r){
        z_radar(0) = min_r;
    }
    z_radar(1) = atan2(py, px);
    z_radar(2) = (px * vx + py * vy) / z_radar(0);
    return z_radar;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
//    if (true){
//        return;
//
//    }
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    VectorXd z_pred = VectorXd(3);
    z_pred(0) = sqrt(px*px + py*py);
    float min_r = 0.0001;
    cout << "R predection " << z_pred(0) << endl;
    if (z_pred(0) <= min_r) {
        cout << "Skipping this measure since it is too small" << endl;
        z_pred(0) = min_r;
        return;
    }
    z_pred(1) = atan2(py, px);
    z_pred(2) = (px*vx + py*vy) / z_pred(0);
//    VectorXd z_pred = GetRadarMeas(x_); //H_ * x_;
    VectorXd y = z - z_pred;

  cout << "before update = " << x_ << endl;
    cout << "z = " << z << endl;
    cout << "z_pred = " << z_pred << endl;


    static const double PI = 3.14159265359;
    while ( y(1) >  PI ) y(1) -= PI;
    while ( y(1) < -PI ) y(1) += PI;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
  cout << "R = " << R_ << endl;
          cout << "H = " << H_ << endl;
           cout << "S = " << S << endl;
           cout << "PHt = " << PHt << endl;
      cout << "K = " << K << endl;
    cout << "y = " << y << endl;

  cout << "after update x = " << x_ << endl;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    /**
     TODO:
     * update the state by using Extended Kalman Filter equations
     */
}
